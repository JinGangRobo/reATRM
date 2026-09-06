// MuJoCo-backed CBoard transport (P2): the simulation subsystem's only backend.
// Drives the robot's motors + IMU with the MuJoCo physics engine while keeping
// the exact same CBoardTransport seam, so hardware models remain unchanged.
//
// Architecture:
//   - One MujocoEngine per robot owns a single mjModel/mjData and a 1 kHz step
//     thread. It applies commanded torques, steps the physics, then streams DJI
//     motor feedback + IMU (gyro/accelerometer sensors) to every attached CBoard
//     through the same receive callbacks a real ATRM C board would use.
//   - A thin MujocoBoardTransport (one per CBoard, i.e. per board USB pid) bridges
//     that CBoard's downlink command frames into per-motor torques.
//
// This translation unit is only compiled when MuJoCo is available (see
// CMakeLists.txt). Robot<->CAN topology is fully config-driven (SimBoardConfig /
// SimMotorSpec in sim_common.hpp, populated from the robot's sim config), so the
// unmodified hardware-model .cpp of any robot runs against this backend as-is.

#if defined(RMCS_SIM_HAS_MUJOCO)

# include <algorithm>
# include <atomic>
# include <chrono>
# include <cmath>
# include <condition_variable>
# include <cstdint>
# include <cstdio>
# include <cstring>
# include <memory>
# include <mutex>
# include <string>
# include <thread>
# include <vector>

# include <mujoco/mujoco.h>
# if defined(RMCS_SIM_HAS_GUI)
#  include <GLFW/glfw3.h>
# endif

# include <librmcs/client/cboard.hpp>
# include <librmcs/client/cboard_transport.hpp>
# include <rclcpp/logging.hpp>

# include "sim_common.hpp"

namespace rmcs_core::simulation {

namespace {

// (Robot topology is config-driven; see sim_common.hpp SimBoardConfig.)

// ---- Engine ----------------------------------------------------------------

class MujocoEngine {
public:
    struct Motor {
        SimMotorSpec spec;
        int actuator_id = -1;
        int joint_id = -1;
        int qpos_adr = -1;
        int dof_adr = -1;
        std::atomic<int> command_raw{0};    // DJI: raw current command
        std::atomic<double> torque_nm{0.0}; // DM: load torque command (N*m)

        Motor() = default;
        Motor(const Motor&) = delete;
        Motor& operator=(const Motor&) = delete;
        Motor(Motor&& other) noexcept
            : spec(std::move(other.spec))
            , actuator_id(other.actuator_id)
            , joint_id(other.joint_id)
            , qpos_adr(other.qpos_adr)
            , dof_adr(other.dof_adr)
            , command_raw(other.command_raw.load())
            , torque_nm(other.torque_nm.load()) {}
        Motor& operator=(Motor&& other) noexcept {
            if (this != &other) {
                spec = std::move(other.spec);
                actuator_id = other.actuator_id;
                joint_id = other.joint_id;
                qpos_adr = other.qpos_adr;
                dof_adr = other.dof_adr;
                command_raw.store(other.command_raw.load());
                torque_nm.store(other.torque_nm.load());
            }
            return *this;
        }
    };

    struct Board {
        librmcs::client::CBoard* cboard = nullptr;
        int pid = 0;
        size_t board_index = 0;
        bool active = false;
        bool dbus = false; // board hosts the DR16
        std::vector<size_t> motor_indices;
        int gyro_adr = -1;
        int acc_adr = -1;
    };

    explicit MujocoEngine(const std::string& model_path) {
        char error[1024] = {0};
        model_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
        if (model_ == nullptr) {
            RCLCPP_FATAL(
                rclcpp::get_logger("Sim"), "MujocoEngine: failed to load '%s': %s",
                model_path.c_str(), error);
            throw std::runtime_error{"MujocoEngine: mj_loadXML failed"};
        }
        data_ = mj_makeData(model_);
        // One pass to initialize derived quantities (xpos/xmat/sensordata...).
        mj_forward(model_, data_);
        RCLCPP_INFO(
            rclcpp::get_logger("Sim"), "MujocoEngine: loaded '%s' (%d joints, %d actuators)",
            model_path.c_str(), model_->njnt, model_->nu);

        running_.store(true);
        thread_ = std::thread{[this]() { step_loop(); }};

# if defined(RMCS_SIM_HAS_GUI)
        if (sim_global_config().gui)
            start_gui();
# endif
    }

    ~MujocoEngine() {
        running_.store(false);
        wake_.notify_all();
        if (thread_.joinable())
            thread_.join();
# if defined(RMCS_SIM_HAS_GUI)
        if (gui_thread_.joinable()) {
            gui_quit_.store(true, std::memory_order::relaxed);
            gui_thread_.join();
        }
# endif
        mj_deleteData(data_);
        mj_deleteModel(model_);
    }

    // Called by a transport.attach() while the model is being constructed; stores
    // the board pointer and resolves all its motors + IMU sensors (taken from
    // the config-driven SimBoardConfig matching `pid`) against the loaded MJCF.
    void attach_board(librmcs::client::CBoard& board, int pid) {
        const SimBoardConfig* cfg = nullptr;
        for (const auto& b : sim_global_config().boards) {
            if (b.pid == pid) {
                cfg = &b;
                break;
            }
        }
        if (cfg == nullptr) {
            RCLCPP_ERROR(
                rclcpp::get_logger("Sim"),
                "MujocoEngine: no SimBoardConfig for board pid %d (check the sim config "
                "'boards')",
                pid);
            return;
        }

        Board b;
        b.cboard = &board;
        b.pid = pid;
        b.active = true;
        b.dbus = cfg->dbus;

        int sensor_gyro = cfg->gyro_sensor.empty()
                            ? -1
                            : mj_name2id(model_, mjOBJ_SENSOR, cfg->gyro_sensor.c_str());
        if (sensor_gyro >= 0)
            b.gyro_adr = model_->sensor_adr[sensor_gyro];
        int sensor_acc = cfg->acc_sensor.empty()
                           ? -1
                           : mj_name2id(model_, mjOBJ_SENSOR, cfg->acc_sensor.c_str());
        if (sensor_acc >= 0)
            b.acc_adr = model_->sensor_adr[sensor_acc];

        {
            std::lock_guard<std::mutex> lock(mutex_);
            b.board_index = boards_.size();
            boards_.push_back(std::move(b));
            for (const auto& m : cfg->motors) {
                SimMotorSpec spec = m;
                if (!motor_is_dm(spec.kind))
                    spec.torque_per_raw = dji_torque_per_raw(spec.kind, spec.reduction, spec.sign);
                Motor motor;
                motor.spec = std::move(spec);
                motor.actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, m.actuator.c_str());
                motor.joint_id = mj_name2id(model_, mjOBJ_JOINT, m.actuator.c_str());
                if (motor.joint_id >= 0) {
                    motor.qpos_adr = model_->jnt_qposadr[motor.joint_id];
                    motor.dof_adr = model_->jnt_dofadr[motor.joint_id];
                }
                if (motor.actuator_id < 0 || motor.joint_id < 0) {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("Sim"),
                        "MujocoEngine: motor '%s' (pid %d) not found in MJCF (act=%d jnt=%d)",
                        m.actuator.c_str(), pid, motor.actuator_id, motor.joint_id);
                }
                motors_.push_back(std::move(motor));
                boards_.back().motor_indices.push_back(motors_.size() - 1);
            }
        }
    }

    void detach_board(librmcs::client::CBoard* board) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& b : boards_) {
            if (b.cboard == board)
                b.active = false;
        }
    }

    const Board* board_by_pid(int pid) const {
        for (const auto& b : boards_) {
            if (b.pid == pid)
                return &b;
        }
        return nullptr;
    }

    // Command setters, indexed by (pid, index inside that board's config list).
    void set_dji_current(int pid, size_t motor_in_board, int raw_current) {
        const auto* b = board_by_pid(pid);
        if (b != nullptr && motor_in_board < b->motor_indices.size())
            motors_[b->motor_indices[motor_in_board]].command_raw.store(
                raw_current, std::memory_order::relaxed);
    }

    void set_dm_torque(int pid, size_t motor_in_board, double torque_nm) {
        const auto* b = board_by_pid(pid);
        if (b != nullptr && motor_in_board < b->motor_indices.size())
            motors_[b->motor_indices[motor_in_board]].torque_nm.store(
                torque_nm, std::memory_order::relaxed);
    }

    // Planar pose (x, y, yaw) of the mobile base's free joint (sim debugging /
    // headless verification that the vehicle actually drives).
    bool get_base_pose(double& x, double& y, double& yaw) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int j = 0; j < model_->njnt; ++j) {
            if (model_->jnt_type[j] != mjJNT_FREE)
                continue;
            const int a = model_->jnt_qposadr[j];
            const double qw = data_->qpos[a + 3];
            const double qx = data_->qpos[a + 4];
            const double qy = data_->qpos[a + 5];
            const double qz = data_->qpos[a + 6];
            x = data_->qpos[a + 0];
            y = data_->qpos[a + 1];
            yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            return true;
        }
        return false;
    }

private:
# if defined(RMCS_SIM_HAS_GUI)
    // ---- optional native MuJoCo (GLFW) viewer ------------------------------
    // Runs on its own thread so a slow/broken GL stack can never stall the
    // 1 kHz physics/control thread. Each frame snapshots the live mjData under
    // the physics mutex and renders the copy (simulate-style decoupling).
    struct GuiPointer {              // stored as the GLFW window user pointer
        mjvCamera* cam = nullptr;
        SimRemote* remote = nullptr; // live simulated DR16 state (shared atomically)
        double last_x = 0.0;
        double last_y = 0.0;
        bool left = false;           // left mouse button held (orbit)
        bool right = false;          // right mouse button held (pan)
        bool key_w = false;          // left stick up    (ch2 y)
        bool key_s = false;          // left stick down  (ch2 y)
        bool key_a = false;          // left stick left  (ch3 x)
        bool key_d = false;          // left stick right (ch3 x)
        bool r_up = false;           // right stick up    (ch0 y)
        bool r_dn = false;           // right stick down  (ch0 y)
        bool r_lf = false;           // right stick left  (ch1 x)
        bool r_rt = false;           // right stick right (ch1 x)
        bool knob_up = false;        // rotary knob increase (']')
        bool knob_dn = false;        // rotary knob decrease ('[')
        double axis_ly = 1024.0;     // smoothed left-stick y (ch2)
        double axis_lx = 1024.0;     // smoothed left-stick x (ch3)
        double axis_ry = 1024.0;     // smoothed right-stick y (ch0)
        double axis_rx = 1024.0;     // smoothed right-stick x (ch1)
        double knob = 1024.0;        // rotary knob value (positional, no spring)
    };

    void start_gui() {
        if (gui_started_)
            return;
        gui_started_ = true;
        gui_quit_.store(false, std::memory_order::relaxed);
        gui_thread_ = std::thread{[this]() { gui_loop(); }};
    }

    void gui_loop() {
        using namespace std::chrono_literals;
        if (!glfwInit()) {
            RCLCPP_WARN(
                rclcpp::get_logger("Sim"), "MujocoEngine GUI: glfwInit failed, running headless");
            return;
        }
        glfwWindowHint(GLFW_SAMPLES, 4);
        glfwWindowHint(GLFW_VISIBLE, 1);
        GLFWwindow* window =
            glfwCreateWindow(1280, 800, "reATRM mini-infantry (MuJoCo sim)", nullptr, nullptr);
        if (window == nullptr) {
            RCLCPP_WARN(
                rclcpp::get_logger("Sim"),
                "MujocoEngine GUI: glfwCreateWindow failed, running headless");
            glfwTerminate();
            return;
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        // Scene / render context / camera (canonical MuJoCo init order).
        mjv_defaultCamera(&gui_cam_);
        mjv_defaultOption(&gui_opt_);
        mjv_defaultScene(&gui_scn_);
        mjr_defaultContext(&gui_con_);
        mjv_makeScene(model_, &gui_scn_, 2000);
        mjr_makeContext(model_, &gui_con_, 150);
        mjv_defaultFreeCamera(model_, &gui_cam_);
        gui_data_ = mj_makeData(model_);

        // Drag-to-orbit (left), drag-to-pan (right), scroll-to-zoom.
        GuiPointer gp;
        gp.cam = &gui_cam_;
        gp.remote = &sim_global_config().remote;
        gp.axis_ly = static_cast<double>(gp.remote->channel2.load(std::memory_order::relaxed));
        gp.axis_lx = static_cast<double>(gp.remote->channel3.load(std::memory_order::relaxed));
        gp.axis_ry = static_cast<double>(gp.remote->channel0.load(std::memory_order::relaxed));
        gp.axis_rx = static_cast<double>(gp.remote->channel1.load(std::memory_order::relaxed));
        gp.knob = static_cast<double>(gp.remote->rotary.load(std::memory_order::relaxed));
        glfwSetWindowUserPointer(window, &gp);
        glfwSetCursorPosCallback(
            window, +[](GLFWwindow* w, double x, double y) {
                auto* g = static_cast<GuiPointer*>(glfwGetWindowUserPointer(w));
                if (g->left) {
                    g->cam->azimuth += static_cast<float>((x - g->last_x) * 0.3);
                    g->cam->elevation = static_cast<float>(
                        std::clamp(g->cam->elevation - (y - g->last_y) * 0.3, -89.0, 89.0));
                } else if (g->right) {
                    const float k = static_cast<float>(0.001 * g->cam->distance);
                    g->cam->lookat[0] -= static_cast<float>(x - g->last_x) * k;
                    g->cam->lookat[1] += static_cast<float>(y - g->last_y) * k;
                }
                g->last_x = x;
                g->last_y = y;
            });
        glfwSetMouseButtonCallback(
            window, +[](GLFWwindow* w, int button, int action, int) {
                auto* g = static_cast<GuiPointer*>(glfwGetWindowUserPointer(w));
                if (button == GLFW_MOUSE_BUTTON_LEFT)
                    g->left = (action == GLFW_PRESS);
                else if (button == GLFW_MOUSE_BUTTON_RIGHT)
                    g->right = (action == GLFW_PRESS);
                double x = 0.0, y = 0.0;
                glfwGetCursorPos(w, &x, &y);
                g->last_x = x;
                g->last_y = y;
            });
        glfwSetScrollCallback(
            window, +[](GLFWwindow* w, double, double yoff) {
                auto* g = static_cast<GuiPointer*>(glfwGetWindowUserPointer(w));
                g->cam->distance *= (yoff > 0.0) ? 0.9f : 1.1f;
            });
        // Keyboard -> full simulated DR16 panel (drives the real control chain via
        // the injected DBUS every tick):
        //   left stick  ch2/ch3 : W/S/A/D or arrow keys (spring-returned)
        //   right stick ch0/ch1 : I/K/J/L               (spring-returned)
        //   rotary knob        : '[' decrease, ']' increase (positional, no spring)
        //   right switch       : 1=UP  2=MIDDLE  3=DOWN
        //   left switch        : 4=UP  5=MIDDLE  6=DOWN
        glfwSetKeyCallback(
            window, +[](GLFWwindow* w, int key, int, int action, int) {
                auto* g = static_cast<GuiPointer*>(glfwGetWindowUserPointer(w));
                const bool pressed = (action == GLFW_PRESS || action == GLFW_REPEAT);
                constexpr auto relaxed = std::memory_order::relaxed;
                if (key == GLFW_KEY_W || key == GLFW_KEY_UP)
                    g->key_w = pressed;
                else if (key == GLFW_KEY_S || key == GLFW_KEY_DOWN)
                    g->key_s = pressed;
                else if (key == GLFW_KEY_A || key == GLFW_KEY_LEFT)
                    g->key_a = pressed;
                else if (key == GLFW_KEY_D || key == GLFW_KEY_RIGHT)
                    g->key_d = pressed;
                else if (key == GLFW_KEY_I)
                    g->r_up = pressed;
                else if (key == GLFW_KEY_K)
                    g->r_dn = pressed;
                else if (key == GLFW_KEY_J)
                    g->r_lf = pressed;
                else if (key == GLFW_KEY_L)
                    g->r_rt = pressed;
                else if (key == GLFW_KEY_LEFT_BRACKET)
                    g->knob_dn = pressed;
                else if (key == GLFW_KEY_RIGHT_BRACKET)
                    g->knob_up = pressed;
                else if (action == GLFW_PRESS) { // 3-position switches (Dr16 codes)
                    if (key == GLFW_KEY_1)
                        g->remote->switch_right.store(1, relaxed); // UP
                    else if (key == GLFW_KEY_2)
                        g->remote->switch_right.store(3, relaxed); // MIDDLE
                    else if (key == GLFW_KEY_3)
                        g->remote->switch_right.store(2, relaxed); // DOWN
                    else if (key == GLFW_KEY_4)
                        g->remote->switch_left.store(1, relaxed);  // UP
                    else if (key == GLFW_KEY_5)
                        g->remote->switch_left.store(3, relaxed);  // MIDDLE
                    else if (key == GLFW_KEY_6)
                        g->remote->switch_left.store(2, relaxed);  // DOWN
                }
            });

        RCLCPP_INFO(
            rclcpp::get_logger("Sim"),
            "MujocoEngine GUI: window opened (GL %s) - DR16 keys: Lstk WASD=aim Rstk IJKL=chassis "
            "knob=[] swR:1up/2mid/3dn swL:4up/5mid/6dn",
            glGetString(GL_VERSION));
        bool scene_logged = false;
        while (!glfwWindowShouldClose(window) && !gui_quit_.load(std::memory_order::relaxed)) {
            glfwPollEvents();

            // Drive the simulated sticks/knob toward their pressed targets, then
            // publish to the atomically shared remote state the physics thread
            // encodes into DBUS every tick. Sticks spring back to center; the knob
            // stays where it was left (positional), like the real remote.
            constexpr double kAxisStep = 90.0; // counts per GUI frame (~60 Hz)
            auto drive_axis = [kAxisStep](double& value, bool up_pressed, bool down_pressed) {
                const double target = up_pressed   ? 1024.0 + 660.0
                                    : down_pressed ? 1024.0 - 660.0
                                                   : 1024.0;
                const double delta = target - value;
                value += std::clamp(delta, -kAxisStep, kAxisStep);
            };
            drive_axis(gp.axis_ly, gp.key_w, gp.key_s); // left stick y  (ch2)
            drive_axis(gp.axis_lx, gp.key_d, gp.key_a); // left stick x  (ch3)
            drive_axis(gp.axis_ry, gp.r_up, gp.r_dn);   // right stick y (ch0)
            drive_axis(gp.axis_rx, gp.r_rt, gp.r_lf);   // right stick x (ch1)
            if (gp.knob_up && !gp.knob_dn)
                gp.knob = std::min(gp.knob + kAxisStep, 1684.0);
            else if (gp.knob_dn && !gp.knob_up)
                gp.knob = std::max(gp.knob - kAxisStep, 364.0);
            constexpr auto relaxed = std::memory_order::relaxed;
            gp.remote->channel0.store(static_cast<int>(std::lround(gp.axis_ry)), relaxed);
            gp.remote->channel1.store(static_cast<int>(std::lround(gp.axis_rx)), relaxed);
            gp.remote->channel2.store(static_cast<int>(std::lround(gp.axis_ly)), relaxed);
            gp.remote->channel3.store(static_cast<int>(std::lround(gp.axis_lx)), relaxed);
            gp.remote->rotary.store(static_cast<int>(std::lround(gp.knob)), relaxed);

            // Snapshot the live physics state under the engine lock.
            {
                std::lock_guard<std::mutex> lock(mutex_);
                mj_copyData(gui_data_, model_, data_);
            }

            int fbw = 0, fbh = 0;
            glfwGetFramebufferSize(window, &fbw, &fbh);
            if (fbw <= 0 || fbh <= 0) { // minimized -> wait for restore
                std::this_thread::sleep_for(10ms);
                continue;
            }
            const mjrRect viewport{0, 0, fbw, fbh};
            mjv_updateScene(model_, gui_data_, &gui_opt_, nullptr, &gui_cam_, mjCAT_ALL, &gui_scn_);
            if (!scene_logged) {
                scene_logged = true;
                RCLCPP_INFO(
                    rclcpp::get_logger("Sim"), "MujocoEngine GUI: first scene has %d geoms",
                    gui_scn_.ngeom);
            }
            mjr_render(viewport, &gui_scn_, &gui_con_);
            auto sw_name = [](int v) {
                return v == 1 ? "UP" : (v == 3 ? "MID" : (v == 2 ? "DN" : "??"));
            };
            char osd_state[160];
            std::snprintf(
                osd_state, sizeof(osd_state),
                "DR16  ch: R(y)=%d R(x)=%d | L(y)=%d L(x)=%d | knob=%d  SW L=%s R=%s",
                static_cast<int>(gp.remote->channel0.load(relaxed)),
                static_cast<int>(gp.remote->channel1.load(relaxed)),
                static_cast<int>(gp.remote->channel2.load(relaxed)),
                static_cast<int>(gp.remote->channel3.load(relaxed)),
                static_cast<int>(gp.remote->rotary.load(relaxed)),
                sw_name(gp.remote->switch_left.load(relaxed)),
                sw_name(gp.remote->switch_right.load(relaxed)));
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, osd_state, nullptr, &gui_con_);
            mjr_overlay(
                mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                "Lstk WASD=aim-shift  Rstk IJKL=drive chassis  knob=[ ]  swR:1up/2mid/3dn  "
                "swL:4up/5mid/6dn",
                nullptr, &gui_con_);
            glfwSwapBuffers(window);
        }

        mjr_freeContext(&gui_con_);
        mjv_freeScene(&gui_scn_);
        mj_deleteData(gui_data_);
        gui_data_ = nullptr;
        glfwDestroyWindow(window);
        glfwTerminate();
        RCLCPP_INFO(rclcpp::get_logger("Sim"), "MujocoEngine GUI: closed");
    }
# endif

    void step_loop() {
        using namespace std::chrono_literals;
        auto next = std::chrono::steady_clock::now();
        size_t tick = 0;

        while (running_.load(std::memory_order::relaxed)) {
            step_physics();
            dispatch(tick);

            ++tick;
            next += std::chrono::nanoseconds(static_cast<long>(1'000'000.0));
            std::this_thread::sleep_until(next);
        }
    }

    // Engine-level kinematic ground: after each physics step the base's planar
    // velocity is set from the four chassis-wheel angular velocities using the
    // SAME forward kinematics as rmcs OmniWheelController, so the (torque-driven)
    // real chassis chain produces exact forward/strafe/spin motion with no
    // ground-contact slip. Applied only when the MJCF has a free-jointed base and
    // the four wheel actuators named wheel_lf/lb/rb/rf.
    void apply_kinematic_ground() {
        const SimGlobalConfig& cfg = sim_global_config();
        double w_lf = 0.0, w_lb = 0.0, w_rb = 0.0, w_rf = 0.0;
        bool found[4] = {false, false, false, false};
        for (auto& m : motors_) {
            if (m.dof_adr < 0)
                continue;
            const std::string& a = m.spec.actuator;
            const double v = data_->qvel[m.dof_adr];
            if (a == "wheel_lf") {
                w_lf = v;
                found[0] = true;
            } else if (a == "wheel_lb") {
                w_lb = v;
                found[1] = true;
            } else if (a == "wheel_rb") {
                w_rb = v;
                found[2] = true;
            } else if (a == "wheel_rf") {
                w_rf = v;
                found[3] = true;
            }
        }
        if (!(found[0] && found[1] && found[2] && found[3]))
            return;
        int fd = -1, fa = -1;
        for (int j = 0; j < model_->njnt; ++j) {
            if (model_->jnt_type[j] == mjJNT_FREE) {
                fd = model_->jnt_dofadr[j]; // free joint: 6 consecutive qvel dofs
                fa = model_->jnt_qposadr[j];
                break;
            }
        }
        if (fd < 0 || fa < 0)
            return;
        if (!ground_z0_ready_) {
            ground_z0_ = data_->qpos[fa + 2];
            ground_z0_ready_ = true;
        }

        // OmniWheelController calculate_chassis_velocity:
        //   v = (-w1-w2+w3+w4, w1-w2-w3+w4, (w1+w2+w3+w4)/(rx+ry)) * (-sqrt2/4*Rw)
        // with w1=left_front, w2=left_back, w3=right_back, w4=right_front.
        const double k = -0.7071067811865476 / 4.0 * cfg.ground_wheel_radius; // -sqrt2/4*Rw
        const double sum_radius = cfg.ground_radius_x + cfg.ground_radius_y;
        const double vx = k * (-w_lf - w_lb + w_rb + w_rf);
        const double vy = k * (w_lf - w_lb - w_rb + w_rf);
        const double wz = k * ((w_lf + w_lb + w_rb + w_rf) / sum_radius);

        data_->qvel[fd + 0] = vx;
        data_->qvel[fd + 1] = vy;
        data_->qvel[fd + 2] = 0.0;                                            // no fall
        data_->qvel[fd + 3] = 0.0;                                            // no roll
        data_->qvel[fd + 4] = 0.0;                                            // no pitch
        data_->qvel[fd + 5] = wz;

        // Flatten the base back onto the ground plane (keep heading).
        const double qw = data_->qpos[fa + 3], qx = data_->qpos[fa + 4];
        const double qy = data_->qpos[fa + 5], qz = data_->qpos[fa + 6];
        const double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        data_->qpos[fa + 2] = ground_z0_;
        data_->qpos[fa + 3] = std::cos(yaw * 0.5);
        data_->qpos[fa + 4] = 0.0;
        data_->qpos[fa + 5] = 0.0;
        data_->qpos[fa + 6] = std::sin(yaw * 0.5);
    }

    void step_physics() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& motor : motors_) {
            if (motor.actuator_id < 0)
                continue;
            if (motor_is_dm(motor.spec.kind)) {
                data_->ctrl[motor.actuator_id] = motor.torque_nm.load(std::memory_order::relaxed);
            } else {
                const int cmd = motor.command_raw.load(std::memory_order::relaxed);
                data_->ctrl[motor.actuator_id] =
                    static_cast<double>(cmd) * motor.spec.torque_per_raw;
            }
        }
        mj_step(model_, data_);
        apply_kinematic_ground();
    }

    void dispatch(size_t tick) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& b : boards_) {
            if (!b.active)
                continue;
            // Stream DJI / DM motor feedback to the owning board over the motor's
            // CAN bus (the model's receive callback routes by can_id).
            for (size_t mi : b.motor_indices) {
                const auto& m = motors_[mi];
                if (m.qpos_adr < 0 || m.dof_adr < 0)
                    continue;
                const double angle = data_->qpos[m.qpos_adr];
                const double vel = data_->qvel[m.dof_adr];
                const uint64_t frame =
                    motor_is_dm(m.spec.kind)
                        ? encode_dm_feedback(
                              angle, vel, m.spec, m.torque_nm.load(std::memory_order::relaxed))
                        : encode_dji_feedback(
                              angle, vel, m.spec, m.command_raw.load(std::memory_order::relaxed));
                if (m.spec.can_bus == 2)
                    b.cboard->can2_receive_callback(
                        static_cast<uint32_t>(m.spec.can_id), frame, false, false, 8);
                else
                    b.cboard->can1_receive_callback(
                        static_cast<uint32_t>(m.spec.can_id), frame, false, false, 8);
            }
            // IMU + simulated DBUS (into the board that hosts the DR16).
            dispatch_imu(b);
            if (b.dbus) {
                std::byte dbus[18];
                make_dbus_frame(sim_global_config().remote, dbus);
                b.cboard->dbus_receive_callback(dbus, 18);
            }
        }
        (void)tick;
    }

    void dispatch_imu(Board& b) {
        if (b.acc_adr < 0)
            return;
        // MuJoCo accelerometer: proper acceleration (m/s^2) in the site frame.
        const double ax = data_->sensordata[b.acc_adr + 0];
        const double ay = data_->sensordata[b.acc_adr + 1];
        const double az = data_->sensordata[b.acc_adr + 2];
        constexpr double kCountsPerG = 5461.0; // 32767/6
        constexpr double kG = 9.80665;
        auto to_acc = [](double v) {
            return static_cast<int16_t>(std::clamp(v / kG * kCountsPerG, -32767.0, 32767.0));
        };
        b.cboard->accelerometer_receive_callback(to_acc(ax), to_acc(ay), to_acc(az));

        if (b.gyro_adr >= 0) {
            // MuJoCo gyro: angular velocity (rad/s) in the site frame.
            const double gx = data_->sensordata[b.gyro_adr + 0];
            const double gy = data_->sensordata[b.gyro_adr + 1];
            const double gz = data_->sensordata[b.gyro_adr + 2];
            constexpr double kCountsPerRadS = 32767.0 * 180.0 / (2000.0 * kPi);
            auto to_gyro = [](double v) {
                return static_cast<int16_t>(std::clamp(v * kCountsPerRadS, -32767.0, 32767.0));
            };
            b.cboard->gyroscope_receive_callback(to_gyro(gx), to_gyro(gy), to_gyro(gz));
        }
    }

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    std::mutex mutex_;
    std::vector<Motor> motors_;
    std::vector<Board> boards_;

    // Kinematic-ground state (base planar height).
    double ground_z0_ = 0.0;
    bool ground_z0_ready_ = false;

    std::atomic<bool> running_{false};
    std::condition_variable wake_;
    std::thread thread_;

# if defined(RMCS_SIM_HAS_GUI)
    bool gui_started_ = false;
    std::atomic<bool> gui_quit_{false};
    std::thread gui_thread_;
    mjData* gui_data_ = nullptr;
    mjvScene gui_scn_; // used after mjv_defaultScene in gui_loop()
    mjrContext gui_con_;
    mjvCamera gui_cam_;
    mjvOption gui_opt_;
# endif
};

std::shared_ptr<MujocoEngine>& engine_instance() {
    static std::shared_ptr<MujocoEngine> instance;
    return instance;
}

// ---- Board transport -------------------------------------------------------

class MujocoBoardTransport final : public librmcs::client::CBoardTransport {
public:
    explicit MujocoBoardTransport(std::shared_ptr<MujocoEngine> engine, int pid)
        : engine_(std::move(engine))
        , pid_(pid) {
        for (const auto& b : sim_global_config().boards) {
            if (b.pid == pid_) {
                config_motors_ = b.motors;
                break;
            }
        }
    }

    void attach(librmcs::client::CBoard& board) override {
        board_ = &board;
        engine_->attach_board(board, pid_);
    }

    void run() override {
        if (board_ == nullptr)
            return;
        // The engine's own thread performs stepping and feedback dispatch; this
        // board event thread simply idles until stop_handling_events().
        RCLCPP_INFO(
            rclcpp::get_logger("Sim"), "MujocoBoardTransport(board pid=%d): attached (%zu motors)",
            pid_, config_motors_.size());
        std::unique_lock<std::mutex> lock(stop_mutex_);
        stop_cv_.wait(lock, [this] { return stop_.load(std::memory_order::relaxed); });
    }

    void stop() override {
        stop_.store(true, std::memory_order::relaxed);
        stop_cv_.notify_all();
    }

    ~MujocoBoardTransport() override {
        if (board_ != nullptr)
            engine_->detach_board(board_);
    }

    void can_transmission(
        uint8_t can_bus, uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
        bool is_remote_transmission, uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
            return;

        // Resolve the frame against this board's config-driven motor binding:
        // either a DM MIT master command on the motor's CAN2 bus, or a DJI
        // 0x200/0x1FF current broadcast on the motor's bus. Engine motor index ==
        // index inside this board's config list (attach pushes them in order).
        for (size_t i = 0; i < config_motors_.size(); ++i) {
            const auto& m = config_motors_[i];
            if (static_cast<int>(can_bus) != m.can_bus)
                continue;
            if (motor_is_dm(m.kind)) {
                if (can_id == static_cast<uint32_t>(m.command_id)) {
                    // The MIT payload torque is motor-frame; `sign` folds the
                    // reversed mounting back to the load frame.
                    const double load_torque = m.sign * dm_decode_command_torque(can_data);
                    engine_->set_dm_torque(pid_, i, load_torque);
                }
            } else {
                const int fbase = m.dji_feedback_base();
                if (fbase != 0 && can_id == static_cast<uint32_t>(m.dji_frame_id())) {
                    // Byte slot inside the frame == can_id - fbase (0..3).
                    const int raw = decode_command_current(can_data, m.can_id, fbase);
                    engine_->set_dji_current(pid_, i, raw);
                }
            }
        }
    }

    void uart1_transmission(const std::byte*, uint8_t) override {}
    void uart2_transmission(const std::byte*, uint8_t) override {}
    void dbus_transmission(const std::byte*, uint8_t) override {}
    void buzzer_transmission(uint8_t) override {}

private:
    std::shared_ptr<MujocoEngine> engine_;
    int pid_ = 0;
    std::vector<SimMotorSpec> config_motors_;
    librmcs::client::CBoard* board_ = nullptr;
    std::atomic<bool> stop_{false};
    std::mutex stop_mutex_;
    std::condition_variable stop_cv_;
};

} // namespace

librmcs::client::CBoardTransport* create_mujoco_transport(int32_t usb_pid) {
    bool found = false;
    for (const auto& b : sim_global_config().boards) {
        if (b.pid == usb_pid) {
            found = true;
            break;
        }
    }
    if (!found) {
        RCLCPP_ERROR(
            rclcpp::get_logger("Sim"),
            "create_mujoco_transport: no SimBoardConfig for board pid %d (check the sim "
            "config 'boards')",
            usb_pid);
        return nullptr;
    }

    auto& engine = engine_instance();
    if (engine == nullptr) {
        if (sim_global_config().model_path.empty()) {
            RCLCPP_ERROR(
                rclcpp::get_logger("Sim"),
                "create_mujoco_transport: no model_path configured (set sim_bootstrap "
                "'model_file')");
            return nullptr;
        }
        engine = std::make_shared<MujocoEngine>(sim_global_config().model_path);
    }
    return new MujocoBoardTransport(engine, usb_pid);
}

// Planar pose of the mobile base's free joint (see MujocoEngine::get_base_pose).
bool engine_get_base_pose(double& x, double& y, double& yaw) {
    auto& engine = engine_instance();
    if (engine == nullptr)
        return false;
    return engine->get_base_pose(x, y, yaw);
}

} // namespace rmcs_core::simulation

#endif // RMCS_SIM_HAS_MUJOCO
