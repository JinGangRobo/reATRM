// MuJoCo-backed CBoard transport (P2). Replaces the P0 fake's integrator
// dynamics with the MuJoCo physics engine while keeping the exact same
// CBoardTransport seam, so hardware models remain unchanged.
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
// CMakeLists.txt). Robot<->CAN topology is still hard-coded for MiniInfantry and
// will be moved to config files in a later phase.

#if defined(RMCS_SIM_HAS_MUJOCO)

# include <algorithm>
# include <atomic>
# include <chrono>
# include <cmath>
# include <condition_variable>
# include <cstdint>
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

// ---- MiniInfantry topology -------------------------------------------------

struct MiniBoardSpec {
    int pid; // board key (usb_pid from the model config)
    std::vector<DjiSimSpec> motors;
    std::string gyro_sensor;
    std::string acc_sensor;
};

MiniBoardSpec build_mini_spec(int pid) {
    MiniBoardSpec spec;
    spec.pid = pid;
    if (pid == 1) {
        // ---- Top board (gimbal pitch + friction wheels) ----
        spec.motors = {
            { "gimbal/left_friction", 0x201, 1.0, -1.0,    0, 0.0,  "left_friction"},
            {"gimbal/right_friction", 0x202, 1.0,  1.0,    0, 0.0, "right_friction"},
            {         "gimbal/pitch", 0x205, 1.0,  1.0, 7556, 0.0,          "pitch"},
        };
        spec.gyro_sensor = "gimbal_gyro";
        spec.acc_sensor = "gimbal_acc";
    } else {
        // ---- Bottom board (chassis + yaw + bullet feeder) ----
        spec.motors = {
            {"chassis/right_front_wheel", 0x201,         268.0 / 17.0, -1.0,    0, 0.0,      "wheel_rf"},
            { "chassis/left_front_wheel", 0x202,         268.0 / 17.0, -1.0,    0, 0.0,      "wheel_lf"},
            {  "chassis/left_back_wheel", 0x203,         268.0 / 17.0, -1.0,    0, 0.0,      "wheel_lb"},
            { "chassis/right_back_wheel", 0x204,         268.0 / 17.0, -1.0,    0, 0.0,      "wheel_rb"},
            {               "gimbal/yaw", 0x206,                  1.0,  1.0, 3606, 0.0,           "yaw"},
            {     "gimbal/bullet_feeder", 0x207, (33.0 / 27.0) * 36.0, -1.0,    0, 0.0, "bullet_feeder"},
        };
        spec.gyro_sensor = "base_gyro";
        spec.acc_sensor = "base_acc";
    }

    // Resolve per-motor device type -> torque-per-raw coefficient.
    for (auto& motor : spec.motors) {
        if (motor.actuator.find("friction") != std::string::npos
            || motor.actuator.rfind("wheel_", 0) == 0) {
            motor.torque_per_raw = dji_torque_per_raw(DjiType::M3508, motor.reduction, motor.sign);
        } else if (motor.actuator == "bullet_feeder") {
            motor.torque_per_raw = dji_torque_per_raw(DjiType::M2006, motor.reduction, motor.sign);
        } else { // pitch / yaw GM6020
            motor.torque_per_raw = dji_torque_per_raw(DjiType::GM6020, motor.reduction, motor.sign);
        }
    }
    return spec;
}

// ---- Engine ----------------------------------------------------------------

class MujocoEngine {
public:
    struct Motor {
        DjiSimSpec spec;
        int actuator_id = -1;
        int joint_id = -1;
        int qpos_adr = -1;
        int dof_adr = -1;
        std::atomic<int> command_raw{0};

        Motor() = default;
        Motor(const Motor&) = delete;
        Motor& operator=(const Motor&) = delete;
        Motor(Motor&& other) noexcept
            : spec(std::move(other.spec))
            , actuator_id(other.actuator_id)
            , joint_id(other.joint_id)
            , qpos_adr(other.qpos_adr)
            , dof_adr(other.dof_adr)
            , command_raw(other.command_raw.load()) {}
        Motor& operator=(Motor&& other) noexcept {
            if (this != &other) {
                spec = std::move(other.spec);
                actuator_id = other.actuator_id;
                joint_id = other.joint_id;
                qpos_adr = other.qpos_adr;
                dof_adr = other.dof_adr;
                command_raw.store(other.command_raw.load());
            }
            return *this;
        }
    };

    struct Board {
        librmcs::client::CBoard* cboard = nullptr;
        int pid = 0;
        size_t board_index = 0;
        bool active = false;
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
    // the board pointer and resolves all names against the loaded MJCF.
    void attach_board(librmcs::client::CBoard& board, int pid, const MiniBoardSpec& spec) {
        Board b;
        b.cboard = &board;
        b.pid = pid;
        b.active = true;

        int sensor_gyro = mj_name2id(model_, mjOBJ_SENSOR, spec.gyro_sensor.c_str());
        if (sensor_gyro >= 0)
            b.gyro_adr = model_->sensor_adr[sensor_gyro];
        int sensor_acc = mj_name2id(model_, mjOBJ_SENSOR, spec.acc_sensor.c_str());
        if (sensor_acc >= 0)
            b.acc_adr = model_->sensor_adr[sensor_acc];

        {
            std::lock_guard<std::mutex> lock(mutex_);
            b.board_index = boards_.size();
            boards_.push_back(std::move(b));
            for (const auto& m : spec.motors) {
                Motor motor;
                motor.spec = m;
                motor.actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, m.actuator.c_str());
                motor.joint_id = mj_name2id(model_, mjOBJ_JOINT, m.actuator.c_str());
                if (motor.joint_id >= 0) {
                    motor.qpos_adr = model_->jnt_qposadr[motor.joint_id];
                    motor.dof_adr = model_->jnt_dofadr[motor.joint_id];
                }
                if (motor.actuator_id < 0 || motor.joint_id < 0) {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("Sim"),
                        "MujocoEngine: motor '%s' not found in MJCF (act=%d jnt=%d)",
                        m.actuator.c_str(), motor.actuator_id, motor.joint_id);
                }
                motor_indices_.push_back(motors_.size());
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

    void set_current(size_t motor_idx, int raw_current) {
        if (motor_idx < motors_.size())
            motors_[motor_idx].command_raw.store(raw_current, std::memory_order::relaxed);
    }

    size_t motor_count() const { return motors_.size(); }
    size_t board_count() const { return boards_.size(); }
    const Board& board(size_t i) const { return boards_[i]; }
    const Motor& motor(size_t i) const { return motors_[i]; }

private:
# if defined(RMCS_SIM_HAS_GUI)
    // ---- optional native MuJoCo (GLFW) viewer ------------------------------
    // Runs on its own thread so a slow/broken GL stack can never stall the
    // 1 kHz physics/control thread. Each frame snapshots the live mjData under
    // the physics mutex and renders the copy (simulate-style decoupling).
    struct GuiPointer { // stored as the GLFW window user pointer
        mjvCamera* cam = nullptr;
        double last_x = 0.0;
        double last_y = 0.0;
        bool left = false;
        bool right = false;
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

        RCLCPP_INFO(
            rclcpp::get_logger("Sim"), "MujocoEngine GUI: window opened (GL %s)",
            glGetString(GL_VERSION));
        bool scene_logged = false;
        while (!glfwWindowShouldClose(window) && !gui_quit_.load(std::memory_order::relaxed)) {
            glfwPollEvents();

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

    void step_physics() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& motor : motors_) {
            const int cmd = motor.command_raw.load(std::memory_order::relaxed);
            data_->ctrl[motor.actuator_id] = static_cast<double>(cmd) * motor.spec.torque_per_raw;
        }
        mj_step(model_, data_);
    }

    void dispatch(size_t tick) {
        std::vector<std::pair<size_t, size_t>> pending; // (board_idx, motor_idx)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (size_t bi = 0; bi < boards_.size(); ++bi) {
                if (!boards_[bi].active)
                    continue;
                for (size_t mi : boards_[bi].motor_indices) {
                    const auto& m = motors_[mi];
                    if (m.qpos_adr < 0 || m.dof_adr < 0)
                        continue;
                    const double angle = data_->qpos[m.qpos_adr];
                    const double vel = data_->qvel[m.dof_adr];
                    const int cmd = m.command_raw.load(std::memory_order::relaxed);
                    const uint64_t frame = encode_dji_feedback(angle, vel, m.spec, cmd);
                    boards_[bi].cboard->can1_receive_callback(
                        static_cast<uint32_t>(m.spec.can_id), frame, false, false, 8);
                }
            }
            // IMU dispatch under the same lock (uses boards_[]).
            for (auto& b : boards_) {
                if (!b.active)
                    continue;
                dispatch_imu(b);
            }
            // Inject the simulated DBUS (remote controller) into the top board,
            // which hosts the Dr16 device in the hardware model.
            for (auto& b : boards_) {
                if (!b.active || b.pid != 1)
                    continue;
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
    std::vector<size_t> motor_indices_;
    std::vector<Board> boards_;

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
        if (pid_ == 1 || pid_ == 2)
            spec_ = build_mini_spec(pid_);
    }

    void attach(librmcs::client::CBoard& board) override {
        board_ = &board;
        engine_->attach_board(board, pid_, spec_);
    }

    void run() override {
        if (board_ == nullptr)
            return;
        // The engine's own thread performs stepping and feedback dispatch; this
        // board event thread simply idles until stop_handling_events().
        RCLCPP_INFO(
            rclcpp::get_logger("Sim"), "MujocoBoardTransport(board pid=%d): attached", pid_);
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
        (void)can_bus;
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
            return;

        uint32_t base = 0;
        if (can_id == 0x200)
            base = 0x201;
        else if (can_id == 0x1FF)
            base = 0x205;
        else
            return;

        for (const auto& m : spec_.motors) {
            if (m.can_id >= static_cast<int>(base) && m.can_id < static_cast<int>(base + 4)) {
                const int raw = decode_command_current(can_data, m.can_id, base);
                // motor index inside the engine == index inside the per-board spec
                // (attach pushed this board's motors in spec order). Store command
                // by (board, index): use a linear search fallback.
                for (size_t bi = 0; bi < engine_->board_count(); ++bi) {
                    const auto& b = engine_->board(bi);
                    if (b.pid == pid_) {
                        for (size_t idx = 0; idx < b.motor_indices.size(); ++idx) {
                            if (idx < spec_.motors.size() && spec_.motors[idx].can_id == m.can_id) {
                                engine_->set_current(b.motor_indices[idx], raw);
                                break;
                            }
                        }
                        break;
                    }
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
    MiniBoardSpec spec_;
    librmcs::client::CBoard* board_ = nullptr;
    std::atomic<bool> stop_{false};
    std::mutex stop_mutex_;
    std::condition_variable stop_cv_;
};

} // namespace

librmcs::client::CBoardTransport* create_mujoco_transport(int32_t usb_pid) {
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
    if (usb_pid != 1 && usb_pid != 2) {
        RCLCPP_ERROR(
            rclcpp::get_logger("Sim"), "create_mujoco_transport: unsupported board pid %d",
            usb_pid);
        return nullptr;
    }
    return new MujocoBoardTransport(engine, usb_pid);
}

} // namespace rmcs_core::simulation

#endif // RMCS_SIM_HAS_MUJOCO
