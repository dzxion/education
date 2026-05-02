# 笔记

[toc]

## 软件

### ODrive

双控实际上只要发波就行，使用高级定时器1和高级定时器8发波
定时器配置频率和中断没有关系，中断需要打开才和定时器的频率一致
M0和M1的定时器不是同步的
定时器触发adc采样，但不一定进中断，也就是说adc采样和中断没关系
TIM8控制的pwm波形为中间对齐，这里设置RCR=2，也就是每（2+1）次更新会中断一次。中断肯定是一次上溢一次下溢
ADC转换结果在TIM8更新中断里读取，不是常用的ADC转换结束触发中断

odrive方案解析：
TIM1、8用于控两个电机
ADC1、ADC用于

```c++
void TIM8_UP_TIM13_IRQHandler(void) {
    COUNT_IRQ(TIM8_UP_TIM13_IRQn);
    
    // Entry into this function happens at 21-23 clock cycles after the timer
    // update event.
    __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);

    // If the corresponding timer is counting up, we just sampled in SVM vector 0, i.e. real current
    // If we are counting down, we just sampled in SVM vector 7, with zero current
    bool counting_down = TIM8->CR1 & TIM_CR1_DIR;

    bool timer_update_missed = (counting_down_ == counting_down);
    if (timer_update_missed) {
        motors[0].disarm_with_error(Motor::ERROR_TIMER_UPDATE_MISSED);
        motors[1].disarm_with_error(Motor::ERROR_TIMER_UPDATE_MISSED);
        return;
    }
    counting_down_ = counting_down;

    timestamp_ += TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1);

    if (!counting_down) {
        TaskTimer::enabled = odrv.task_timers_armed_;
        // Run sampling handlers and kick off control tasks when TIM8 is
        // counting up.
        odrv.sampling_cb();
        NVIC->STIR = ControlLoop_IRQn;
    } else {
        // Tentatively reset all PWM outputs to 50% duty cycles. If the control
        // loop handler finishes in time then these values will be overridden
        // before they go into effect.
        TIM1->CCR1 =
        TIM1->CCR2 =
        TIM1->CCR3 =
        TIM8->CCR1 =
        TIM8->CCR2 =
        TIM8->CCR3 =
            TIM_1_8_PERIOD_CLOCKS / 2;
    }
}

/**
 * @brief Updates the phase PWM timings unless the motor is disarmed.
 *
 * If the motor is armed, the PWM timings come into effect at the next update
 * event (and are enabled if they weren't already), unless the motor is disarmed
 * prior to that.
 * 
 * @param tentative: If true, the update is not counted as "refresh".
 */
void Motor::apply_pwm_timings(uint16_t timings[3], bool tentative) {
    CRITICAL_SECTION() {
        if (odrv.config_.enable_brake_resistor && !brake_resistor_armed) {
            disarm_with_error(ERROR_BRAKE_RESISTOR_DISARMED);
        }

        TIM_HandleTypeDef* htim = timer_;
        TIM_TypeDef* tim = htim->Instance;
        tim->CCR1 = timings[0];
        tim->CCR2 = timings[1];
        tim->CCR3 = timings[2];
        
        if (!tentative) {
            if (is_armed_) {
                // Set the Automatic Output Enable so that the Master Output Enable
                // bit will be automatically enabled on the next update event.
                tim->BDTR |= TIM_BDTR_AOE;
            }
        }
        
        // If a timer update event occurred just now while we were updating the
        // timings, we can't be sure what values the shadow registers now contain,
        // so we must disarm the motor.
        // (this also protects against the case where the update interrupt has too
        // low priority, but that should not happen)
        //if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE)) {
        //    disarm_with_error(ERROR_CONTROL_DEADLINE_MISSED);
        //}
    }
}

class Controller : public ODriveIntf::ControllerIntf {
public:
    struct Anticogging_t {
        uint32_t index = 0;
        float cogging_map[3600];
        bool pre_calibrated = false;
        bool calib_anticogging = false;
        float calib_pos_threshold = 1.0f;
        float calib_vel_threshold = 1.0f;
        float cogging_ratio = 1.0f;
        bool anticogging_enabled = true;
    };

    struct Autotuning_t {
        float frequency = 0.0f;
        float pos_amplitude = 0.0f;
        float vel_amplitude = 0.0f;
        float torque_amplitude = 0.0f;
    };

    struct Config_t {
        ControlMode control_mode = CONTROL_MODE_POSITION_CONTROL;  //see: ControlMode_t
        InputMode input_mode = INPUT_MODE_PASSTHROUGH;             //see: InputMode_t
        float pos_gain = 20.0f;                  // [(turn/s) / turn]
        float vel_gain = 1.0f / 6.0f;            // [Nm/(turn/s)]
        // float vel_gain = 0.2f / 200.0f,       // [Nm/(rad/s)] <sensorless example>
        float vel_integrator_gain = 2.0f / 6.0f; // [Nm/(turn/s * s)]
        float vel_limit = 2.0f;                  // [turn/s] Infinity to disable.
        float vel_limit_tolerance = 1.2f;        // ratio to vel_lim. Infinity to disable.
        float vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.
        float vel_ramp_rate = 1.0f;              // [(turn/s) / s]
        float torque_ramp_rate = 0.01f;          // Nm / sec
        bool circular_setpoints = false;
        float circular_setpoint_range = 1.0f;    // Circular range when circular_setpoints is true. [turn]
        uint32_t steps_per_circular_range = 1024;
        float inertia = 0.0f;                    // [Nm/(turn/s^2)]
        float input_filter_bandwidth = 2.0f;     // [1/s]
        float homing_speed = 0.25f;              // [turn/s]
        Anticogging_t anticogging;
        float gain_scheduling_width = 10.0f;
        bool enable_gain_scheduling = false;
        bool enable_vel_limit = true;
        bool enable_overspeed_error = true;
        bool enable_torque_mode_vel_limit = true;  // enable velocity limit in current control mode (requires a valid velocity estimator)
        uint8_t axis_to_mirror = -1;
        float mirror_ratio = 1.0f;
        float torque_mirror_ratio = 0.0f;
        uint8_t load_encoder_axis = -1;  // default depends on Axis number and is set in load_configuration(). Set to -1 to select sensorless estimator.
        float mechanical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for mechanical power for spinout detction
        float electrical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for electrical power for spinout detection
        float spinout_electrical_power_threshold = 10.0f; // [W] electrical power threshold for spinout detection
        float spinout_mechanical_power_threshold = -10.0f; // [W] mechanical power threshold for spinout detection

        // custom setters
        Controller* parent;
        void set_input_filter_bandwidth(float value) { input_filter_bandwidth = value; parent->update_filter_gains(); }
        void set_steps_per_circular_range(uint32_t value) { steps_per_circular_range = value > 0 ? value : steps_per_circular_range; }
        void set_control_mode(ControlMode value) { control_mode = value; parent->control_mode_updated(); }
    };

    
    bool apply_config();

    void reset();
    void set_error(Error error);

    constexpr void input_pos_updated() {
        input_pos_updated_ = true;
    }
    bool control_mode_updated();
    void set_input_pos_and_steps(float pos);

    bool select_encoder(size_t encoder_num);

    // Trajectory-Planned control
    void move_to_pos(float goal_point);
    void move_incremental(float displacement, bool from_goal_point);
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    float remove_anticogging_bias();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);
    
    float get_anticogging_value(uint32_t index) {
        return (index < 3600) ? config_.anticogging.cogging_map[index] : 0.0f;
    }

    void update_filter_gains();
    bool update();

    Config_t config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error error_ = ERROR_NONE;
    float last_error_time_ = 0.0f;

    // Inputs
    InputPort<float> pos_estimate_linear_src_;
    InputPort<float> pos_estimate_circular_src_;
    InputPort<float> vel_estimate_src_;
    InputPort<float> pos_wrap_src_; 

    float pos_setpoint_ = 0.0f; // [turns]
    float vel_setpoint_ = 0.0f; // [turn/s]
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_torque_ = 0.0f;    // [Nm]
    float torque_setpoint_ = 0.0f;  // [Nm]

    float input_pos_ = 0.0f;     // [turns]
    float input_vel_ = 0.0f;     // [turn/s]
    float input_torque_ = 0.0f;  // [Nm]
    float input_filter_kp_ = 0.0f;
    float input_filter_ki_ = 0.0f;

    Autotuning_t autotuning_;
    float autotuning_phase_ = 0.0f;
    
    bool input_pos_updated_ = false;
    
    bool trajectory_done_ = true;

    bool anticogging_valid_ = false;
    float mechanical_power_ = 0.0f; // [W]
    float electrical_power_ = 0.0f; // [W]

    // Outputs
    OutputPort<float> torque_output_ = 0.0f;

    // custom setters
    void set_input_pos(float value) { set_input_pos_and_steps(value); input_pos_updated(); }
};

//控制器8k运行，根据配置进行模式切换
bool Controller::update() {
    //获取传感器数据
    std::optional<float> pos_estimate_linear = pos_estimate_linear_src_.present();
    std::optional<float> pos_estimate_circular = pos_estimate_circular_src_.present();
    std::optional<float> pos_wrap = pos_wrap_src_.present();
    std::optional<float> vel_estimate = vel_estimate_src_.present();

    std::optional<float> anticogging_pos_estimate = axis_->encoder_.pos_estimate_.present();
    std::optional<float> anticogging_vel_estimate = axis_->encoder_.vel_estimate_.present();

    if (axis_->step_dir_active_) {
        if (config_.circular_setpoints) {
            if (!pos_wrap.has_value()) {
                set_error(ERROR_INVALID_CIRCULAR_RANGE);
                return false;
            }
            input_pos_ = (float)(axis_->steps_ % config_.steps_per_circular_range) * (*pos_wrap / (float)(config_.steps_per_circular_range));
        } else {
            input_pos_ = (float)(axis_->steps_) / (float)(config_.steps_per_circular_range);
        }
    }

    if (config_.anticogging.calib_anticogging) {
        if (!anticogging_pos_estimate.has_value() || !anticogging_vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // non-blocking
        anticogging_calibration(*anticogging_pos_estimate, *anticogging_vel_estimate);
    }

    // TODO also enable circular deltas for 2nd order filter, etc.
    if (config_.circular_setpoints) {
        if (!pos_wrap.has_value()) {
            set_error(ERROR_INVALID_CIRCULAR_RANGE);
            return false;
        }
        input_pos_ = fmodf_pos(input_pos_, *pos_wrap);
    }

    // Update inputs
    // input_mode指的是怎么给参考信号
    switch (config_.input_mode) {
        case INPUT_MODE_INACTIVE: {
            // do nothing
        } break;
        case INPUT_MODE_PASSTHROUGH: {
            pos_setpoint_ = input_pos_;
            vel_setpoint_ = input_vel_;
            torque_setpoint_ = input_torque_; 
        } break;
        case INPUT_MODE_VEL_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.vel_ramp_rate);
            float full_step = input_vel_ - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            torque_setpoint_ = (step / current_meas_period) * config_.inertia;
        } break;
        case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.torque_ramp_rate);
            float full_step = input_torque_ - torque_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            torque_setpoint_ += step;
        } break;
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            if (config_.circular_setpoints) {
                if (!pos_wrap.has_value()) {
                    set_error(ERROR_INVALID_CIRCULAR_RANGE);
                    return false;
                }
                delta_pos = wrap_pm(delta_pos, *pos_wrap);
            }
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            torque_setpoint_ = accel * config_.inertia; // Accel
            vel_setpoint_ += current_meas_period * accel; // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
        } break;
        case INPUT_MODE_MIRROR: {
            if (config_.axis_to_mirror < AXIS_COUNT) {
                std::optional<float> other_pos = axes[config_.axis_to_mirror].encoder_.pos_estimate_.present();
                std::optional<float> other_vel = axes[config_.axis_to_mirror].encoder_.vel_estimate_.present();
                std::optional<float> other_torque = axes[config_.axis_to_mirror].controller_.torque_output_.present();

                if (!other_pos.has_value() || !other_vel.has_value() || !other_torque.has_value()) {
                    set_error(ERROR_INVALID_ESTIMATE);
                    return false;
                }

                pos_setpoint_ = *other_pos * config_.mirror_ratio;
                vel_setpoint_ = *other_vel * config_.mirror_ratio;
                torque_setpoint_ = *other_torque * config_.torque_mirror_ratio;
            } else {
                set_error(ERROR_INVALID_MIRROR_AXIS);
                return false;
            }
        } break;
        // case INPUT_MODE_MIX_CHANNELS: {
        //     // NOT YET IMPLEMENTED
        // } break;
        case INPUT_MODE_TRAP_TRAJ: {
            if(input_pos_updated_){
                move_to_pos(input_pos_);
                input_pos_updated_ = false;
            }
            // Avoid updating uninitialized trajectory
            if (trajectory_done_)
                break;
            
            if (axis_->trap_traj_.t_ > axis_->trap_traj_.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
                pos_setpoint_ = axis_->trap_traj_.Xf_;
                vel_setpoint_ = 0.0f;
                torque_setpoint_ = 0.0f;
                trajectory_done_ = true;
            } else {
                TrapezoidalTrajectory::Step_t traj_step = axis_->trap_traj_.eval(axis_->trap_traj_.t_);
                pos_setpoint_ = traj_step.Y;
                vel_setpoint_ = traj_step.Yd;
                torque_setpoint_ = traj_step.Ydd * config_.inertia;
                axis_->trap_traj_.t_ += current_meas_period;
            }
            anticogging_pos_estimate = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
        } break;
        case INPUT_MODE_TUNING: {
            autotuning_phase_ = wrap_pm_pi(autotuning_phase_ + (2.0f * M_PI * autotuning_.frequency * current_meas_period));
            float c = our_arm_cos_f32(autotuning_phase_);
            float s = our_arm_sin_f32(autotuning_phase_);
            pos_setpoint_ = input_pos_ + autotuning_.pos_amplitude * s; // + pos_amp_c * c
            vel_setpoint_ = input_vel_ + autotuning_.vel_amplitude * c;
            torque_setpoint_ = input_torque_ + autotuning_.torque_amplitude * -s;
        } break;
        default: {
            set_error(ERROR_INVALID_INPUT_MODE);
            return false;
        }
        
    }

    // Never command a setpoint beyond its limit
    if(config_.enable_vel_limit) {
        vel_setpoint_ = std::clamp(vel_setpoint_, -config_.vel_limit, config_.vel_limit);
    }
    const float Tlim = axis_->motor_.max_available_torque();
    torque_setpoint_ = std::clamp(torque_setpoint_, -Tlim, Tlim);

    /************位置闭环***********/
    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;//位置闭环给参考速度？？？
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {//位置控制模式才会进入
        float pos_err;

        if (config_.circular_setpoints) {
            if (!pos_estimate_circular.has_value() || !pos_wrap.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap);
            // Circular delta
            pos_err = pos_setpoint_ - *pos_estimate_circular;
            pos_err = wrap_pm(pos_err, *pos_wrap);
        } else {
            if (!pos_estimate_linear.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            pos_err = pos_setpoint_ - *pos_estimate_linear;
        }

        vel_des += config_.pos_gain * pos_err;
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    }
    /************位置闭环***********/

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (config_.enable_vel_limit) {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    if (config_.enable_overspeed_error) {  // 0.0f to disable
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        if (std::abs(*vel_estimate) > config_.vel_limit_tolerance * vel_lim) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM)
    float vel_gain = config_.vel_gain;
    float vel_integrator_gain = config_.vel_integrator_gain;
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        float effective_flux = axis_->acim_estimator_.rotor_flux_;
        float minflux = axis_->motor_.config_.acim_gain_min_flux;
        if (std::abs(effective_flux) < minflux)
            effective_flux = std::copysignf(minflux, effective_flux);
        vel_gain /= effective_flux;
        vel_integrator_gain /= effective_flux;
        // TODO: also scale the integral value which is also changing units.
        // (or again just do control in torque units)
    }

    // Velocity control
    float torque = torque_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_valid_ && config_.anticogging.anticogging_enabled) {
        if (!anticogging_pos_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        float anticogging_pos = *anticogging_pos_estimate / axis_->encoder_.getCoggingRatio();
        torque += config_.anticogging.cogging_map[std::clamp(mod((int)anticogging_pos, 3600), 0, 3600)];
    }

    /************速度闭环***********/
    float v_err = 0.0f;
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {//位置或者速度控制模式才会进来
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate;
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting
        torque += vel_integrator_torque_;
    }
    /************速度闭环***********/

    // Velocity limiting in current mode
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL && config_.enable_torque_mode_vel_limit) {
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque = limitVel(config_.vel_limit, *vel_estimate, vel_gain, torque);
    }

    // Torque limiting
    bool limited = false;
    if (torque > Tlim) {
        limited = true;
        torque = Tlim;
    }
    if (torque < -Tlim) {
        limited = true;
        torque = -Tlim;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } else {
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
        }
        // integrator limiting to prevent windup 
        vel_integrator_torque_ = std::clamp(vel_integrator_torque_, -config_.vel_integrator_limit, config_.vel_integrator_limit);
    }

    float ideal_electrical_power = 0.0f;
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL) {
        ideal_electrical_power = axis_->motor_.current_control_.power_ - \
            SQ(axis_->motor_.current_control_.Iq_measured_) * 1.5f * axis_->motor_.config_.phase_resistance - \
            SQ(axis_->motor_.current_control_.Id_measured_) * 1.5f * axis_->motor_.config_.phase_resistance;
    }
    else {
        ideal_electrical_power = axis_->motor_.current_control_.power_;
    }
    mechanical_power_ += config_.mechanical_power_bandwidth * current_meas_period * (torque * *vel_estimate * M_PI * 2.0f - mechanical_power_);
    electrical_power_ += config_.electrical_power_bandwidth * current_meas_period * (ideal_electrical_power - electrical_power_);

    // Spinout check
    // If mechanical power is negative (braking) and measured power is positive, something is wrong
    // This indicates that the controller is trying to stop, but torque is being produced.
    // Usually caused by an incorrect encoder offset
    if (mechanical_power_ < config_.spinout_mechanical_power_threshold && electrical_power_ > config_.spinout_electrical_power_threshold) {
        set_error(ERROR_SPINOUT_DETECTED);
        return false;
    }

    torque_output_ = torque;//这个是最后输出

    // TODO: this is inconsistent with the other errors which are sticky.
    // However if we make ERROR_INVALID_ESTIMATE sticky then it will be
    // confusing that a normal sequence of motor calibration + encoder
    // calibration would leave the controller in an error state.
    error_ &= ~ERROR_INVALID_ESTIMATE;
    return true;
}
```

### APM

#### PID

```c++
// Computes the P controller output given a target and measurement.
// Applies position error clamping based on configured limits.
// Optionally constrains output slope using the sqrt_controller.
Vector2f AC_P_2D::update_all(Vector2p &target, const Vector2p &measurement)
{
    // Compute vector error between target and measurement (NED frame)
    _error = (target - measurement).tofloat();

    // Limit error vector length to prevent exceeding output constraints
    if (is_positive(_error_max) && _error.limit_length(_error_max)) {
        target = measurement + _error.topostype();
    }

    // Use sqrt_controller to limit output and/or its derivative
    return sqrt_controller(_error, _kp, _D1_max, 0.0);
}
```

#### 位置控制

```c++
// Runs the NE-axis position controller, computing output acceleration from position and velocity errors.
    // Uses P and PID controllers to generate corrections which are added to feedforward velocity/acceleration.
    // Requires all desired targets to be pre-set using the input_* or set_* methods.
    void update_NE_controller();

// Runs the vertical (U-axis) position controller.
    // Computes output acceleration based on position and velocity errors using PID correction.
    // Feedforward velocity and acceleration are combined with corrections to produce a smooth vertical command.
    // Desired position, velocity, and acceleration must be set before calling.
    void update_U_controller();

// Uses P and PID controllers to generate corrections which are added to feedforward velocity/acceleration.
// Requires all desired targets to be pre-set using the input_* or set_* methods.
void AC_PosControl::update_NE_controller()
{
    // check for ekf xy position reset
    // 检查EKF（扩展卡尔曼滤波器）是否发生了位置重置。当EKF检测到重大错误或重新初始化时，会重置位置估计。控制器需要知道这个重置，以避免基于旧位置数据产生错误指令。
    handle_ekf_NE_reset();

    // Check for position control time out
    if (!is_active_NE()) {
        init_NE_controller();
        if (has_good_timing()) {
            // call internal error because initialisation has not been done
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }
    _last_update_ne_ticks = AP::scheduler().ticks32();

    float ahrsGndSpdLimit, ahrsControlScaleXY;
    AP::ahrs().getControlLimits(ahrsGndSpdLimit, ahrsControlScaleXY);

    // Update lateral position, velocity, and acceleration offsets using path shaping
    update_offsets_NE();

    // Position Controller

    // Combine position target with active NE offset to get absolute target
    _pos_target_neu_m.xy() = _pos_desired_neu_m.xy() + _pos_offset_neu_m.xy();

    // determine the combined position of the actual position and the disturbance from system ID mode
    // calculate the target velocity correction
    Vector2p comb_pos_ne_m = _pos_estimate_neu_m.xy();
    comb_pos_ne_m += _disturb_pos_ne_m.topostype();

    // Run P controller to compute velocity setpoint from position error
    Vector2f vel_target_ne_ms = _p_pos_ne_m.update_all(_pos_target_neu_m.xy(), comb_pos_ne_m);
    _pos_desired_neu_m.xy() = _pos_target_neu_m.xy() - _pos_offset_neu_m.xy();

    // Velocity Controller

    // Apply AHRS scaling (e.g. for optical flow noise compensation)
    vel_target_ne_ms *= ahrsControlScaleXY;
    vel_target_ne_ms *= _ne_control_scale_factor;

    _vel_target_neu_ms.xy() = vel_target_ne_ms;
    _vel_target_neu_ms.xy() += _vel_desired_neu_ms.xy() + _vel_offset_neu_ms.xy();

    // Velocity Controller

    // determine the combined velocity of the actual velocity and the disturbance from system ID mode
    Vector2f comb_vel_ne_ms = _vel_estimate_neu_ms.xy();
    comb_vel_ne_ms += _disturb_vel_ne_ms;

    // Run velocity PID controller and scale result for control authority
    Vector2f accel_target_ne_mss = _pid_vel_ne_cm.update_all(_vel_target_neu_ms.xy() * 100.0, comb_vel_ne_ms * 100.0, _dt_s, _limit_vector_neu.xy()) * 0.01;

    // Acceleration Controller
    
    // Apply AHRS scaling again to correct for measurement distortions
    accel_target_ne_mss *= ahrsControlScaleXY;
    accel_target_ne_mss *= _ne_control_scale_factor;

    _ne_control_scale_factor = 1.0;

    // pass the correction acceleration to the target acceleration output
    _accel_target_neu_mss.xy() = accel_target_ne_mss;
    _accel_target_neu_mss.xy() += _accel_desired_neu_mss.xy() + _accel_offset_neu_mss.xy();

    // limit acceleration using maximum lean angles
    const float angle_max_rad = MIN(_attitude_control.get_althold_lean_angle_max_rad(), get_lean_angle_max_rad());
    const float accel_max_mss = angle_rad_to_accel_mss(angle_max_rad);
    // Save unbounded target for use in "limited" check (not unit-consistent with z!)
    _limit_vector_neu.xy() = _accel_target_neu_mss.xy();
    if (!limit_accel_xy(_vel_desired_neu_ms.xy(), _accel_target_neu_mss.xy(), accel_max_mss)) {
        // _accel_target_neu_mss was not limited so we can zero the xy limit vector
        _limit_vector_neu.xy().zero();
    }

    // Convert acceleration to roll/pitch angle targets (used by attitude controller)
    accel_NE_mss_to_lean_angles_rad(_accel_target_neu_mss.x, _accel_target_neu_mss.y, _roll_target_rad, _pitch_target_rad);

    // Update yaw and yaw rate targets to match heading of motion
    calculate_yaw_and_rate_yaw();

    // reset the disturbance from system ID mode to zero
    _disturb_pos_ne_m.zero();
    _disturb_vel_ne_ms.zero();
}

// Runs the vertical (U-axis) position controller.
// Computes output acceleration based on position and velocity errors using PID correction.
// Feedforward velocity and acceleration are combined with corrections to produce a smooth vertical command.
// Desired position, velocity, and acceleration must be set before calling.
void AC_PosControl::update_U_controller()
{
    // check for ekf z-axis position reset
    handle_ekf_U_reset();

    // Check for z_controller time out
    if (!is_active_U()) {
        init_U_controller();
        if (has_good_timing()) {
            // call internal error because initialisation has not been done
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }
    _last_update_u_ticks = AP::scheduler().ticks32();

    // Update vertical offset targets and terrain estimate
    update_offsets_U();
    update_terrain();

    // Position Controller

    // Combine desired + offset + terrain for final position target
    _pos_target_neu_m.z = _pos_desired_neu_m.z + _pos_offset_neu_m.z + _pos_terrain_u_m;

    // P controller: convert position error to velocity target
    _vel_target_neu_ms.z = _p_pos_u_m.update_all(_pos_target_neu_m.z, _pos_estimate_neu_m.z);
    _vel_target_neu_ms.z *= AP::ahrs().getControlScaleZ();

    _pos_desired_neu_m.z = _pos_target_neu_m.z - (_pos_offset_neu_m.z + _pos_terrain_u_m);

    // add feed forward component
    _vel_target_neu_ms.z += _vel_desired_neu_ms.z + _vel_offset_neu_ms.z + _vel_terrain_u_ms;

    // Velocity Controller

    // PID controller: convert velocity error to acceleration
    _accel_target_neu_mss.z = _pid_vel_u_cm.update_all(_vel_target_neu_ms.z * 100.0, _vel_estimate_neu_ms.z * 100.0, _dt_s, _motors.limit.throttle_lower, _motors.limit.throttle_upper) * 0.01;
    _accel_target_neu_mss.z *= AP::ahrs().getControlScaleZ();

    // add feed forward component
    _accel_target_neu_mss.z += _accel_desired_neu_mss.z + _accel_offset_neu_mss.z + _accel_terrain_u_mss;

    // Acceleration Controller

    // Gravity-compensated vertical acceleration measurement (positive = up)
    const float measured_accel_u_mss = get_measured_accel_U_mss();

    // Ensure integrator can produce enough thrust to overcome hover throttle
    if (_motors.get_throttle_hover() * 1000.0 > _pid_accel_u_cm_to_kt.imax()) {
        _pid_accel_u_cm_to_kt.set_imax(_motors.get_throttle_hover() * 1000.0);
    }
    float thr_out;
    if (_vibe_comp_enabled) {
        // Use vibration-resistant throttle estimator (feedforward + scaled integrator)
        thr_out = get_throttle_with_vibration_override();
    } else {
        // Standard PID update using vertical acceleration error
        thr_out = _pid_accel_u_cm_to_kt.update_all(_accel_target_neu_mss.z * 100.0, measured_accel_u_mss * 100.0, _dt_s, (_motors.limit.throttle_lower || _motors.limit.throttle_upper)) * 0.001;
        // Include FF contribution to reduce delay
        thr_out += _pid_accel_u_cm_to_kt.get_ff() * 0.001;
    }
    thr_out += _motors.get_throttle_hover();

    // Actuator commands

    // Send final throttle output to attitude controller (includes angle boost)
    _attitude_control.set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ);

    // Check for vertical controller health

    // Update health indicator based on error magnitude vs configured speed range
    float error_ratio = _pid_vel_u_cm.get_error() * 0.01 / _vel_max_down_ms;
    _vel_u_control_ratio += _dt_s * 0.1f * (0.5 - error_ratio);
    _vel_u_control_ratio = constrain_float(_vel_u_control_ratio, 0.0f, 2.0f);

    // set vertical component of the limit vector
    if (_motors.limit.throttle_upper) {
        _limit_vector_neu.z = 1.0f;
    } else if (_motors.limit.throttle_lower) {
        _limit_vector_neu.z = -1.0f;
    } else {
        _limit_vector_neu.z = 0.0f;
    }
}
```

#### TECS

```c++
class AP_TECS {
public:
    AP_TECS(AP_AHRS &ahrs, const AP_FixedWing &parms, const AP_Landing &landing, const uint32_t log_bitmask)
        : _ahrs(ahrs)
        , aparm(parms)
        , _landing(landing)
        , _log_bitmask(log_bitmask)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_TECS);

    // Update of the estimated height and height rate internal state
    // Update of the inertial speed rate internal state
    // Should be called at 50Hz or greater
    void update_50hz(void);

    // Update the control loop calculations
    // Do not call slower than 10Hz or faster than 500Hz
    void update_pitch_throttle(int32_t hgt_dem_cm,
                               int32_t EAS_dem_cm,
                               enum AP_FixedWing::FlightStage flight_stage,
                               float distance_beyond_land_wp,
                               int32_t ptchMinCO_cd,
                               int16_t throttle_nudge,
                               float hgt_afe,
                               float load_factor);

    // demanded throttle in percentage
    // should return -100 to 100, usually positive unless reverse thrust is enabled via _THRminf < 0
    float get_throttle_demand(void) {
        return _throttle_dem * 100.0f;
    }

    // demanded pitch angle in centi-degrees
    // should return between -9000 to +9000
    int32_t get_pitch_demand(void) {
        return int32_t(_pitch_dem * 5729.5781f);
    }

    // Rate of change of velocity along X body axis in m/s^2
    float get_VXdot(void) {
        return _vel_dot;
    }

    // return current target airspeed
    float get_target_airspeed(void) const {
        return _TAS_dem_adj / _ahrs.get_EAS2TAS();
    }

    // return maximum climb rate
    float get_max_climbrate(void) const {
        return _maxClimbRate;
    }

    // return maximum sink rate (+ve number down)
    float get_max_sinkrate(void) const {
        return _maxSinkRate;
    }
    
    // added to let SoaringContoller reset pitch integrator to zero
    void reset_pitch_I(void) {
        _integSEBdot = 0.0f;
        _integKE = 0.0f;
    }

    // reset throttle integrator
    void reset_throttle_I(void) {
        _integTHR_state = 0.0;
    }

    // return landing sink rate
    float get_land_sinkrate(void) const {
        return _land_sink;
    }

    // return landing airspeed
    float get_land_airspeed(void) const {
        return _landAirspeed;
    }

    // return height rate demand, in m/s
    float get_height_rate_demand(void) const {
        return _hgt_rate_dem;
    }

    // set path_proportion
    void set_path_proportion(float path_proportion) {
        _path_proportion = constrain_float(path_proportion, 0.0f, 1.0f);
    }

    // set soaring flag
    void set_gliding_requested_flag(bool gliding_requested) {
        _flags.gliding_requested = gliding_requested;
    }

    // set propulsion failed flag
    void set_propulsion_failed_flag(bool propulsion_failed) {
        _flags.propulsion_failed = propulsion_failed;
    }


    // set pitch max limit in degrees
    void set_pitch_max_limit(int8_t pitch_limit) {
        _pitch_max_limit = pitch_limit;
    }

    // force use of synthetic airspeed for one loop
    void use_synthetic_airspeed(void) {
        _use_synthetic_airspeed_once = true;
    }

    // reset on next loop
    void reset(void) {
        _need_reset = true;
    }

    // this supports the TECS_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    // Last time update_50Hz was called
    uint64_t _update_50hz_last_usec;//update_50Hz函数上次更新时间（us）

    // Last time update_speed was called
    uint64_t _update_speed_last_usec;

    // Last time update_pitch_throttle was called
    uint64_t _update_pitch_throttle_last_usec;//上次运行update_pitch_throttle的时间

    // reference to the AHRS object
    AP_AHRS &_ahrs;

    const AP_FixedWing &aparm;

    // reference to const AP_Landing to access it's params
    const AP_Landing &_landing;

    // Logging  bitmask
    const uint32_t _log_bitmask;

    // TECS tuning parameters
    AP_Float _hgtCompFiltOmega;
    AP_Float _spdCompFiltOmega;
    AP_Float _maxClimbRate;//最大爬升率（参数）
    AP_Float _minSinkRate;
    AP_Float _maxSinkRate;//最大下降率（参数）
    AP_Float _timeConst;
    AP_Float _landTimeConst;
    AP_Float _ptchDamp;
    AP_Float _land_pitch_damp;
    AP_Float _landDamp;
    AP_Float _thrDamp;//油门控制阻尼（参数，对_STE_error变化率的阻尼）
    AP_Float _land_throttle_damp;
    AP_Float _integGain;
    AP_Float _integGain_takeoff;
    AP_Float _integGain_land;
    AP_Float _vertAccLim;
    AP_Float _rollComp;
    AP_Float _spdWeight;//动能权重值（参数）
    AP_Float _spdWeightLand;
    AP_Float _landThrottle;
    AP_Float _landAirspeed;
    AP_Float _land_sink;
    AP_Float _land_sink_rate_change;
    AP_Int8  _pitch_max;
    AP_Int8  _pitch_min;
    AP_Int8  _land_pitch_max;
    AP_Float _maxSinkRate_approach;
    AP_Int32 _options;
    AP_Float _flare_holdoff_hgt;
    AP_Float _hgt_dem_tconst;

    enum {
        OPTION_GLIDER_ONLY=(1<<0)
    };

    AP_Float _pitch_ff_v0;
    AP_Float _pitch_ff_k;
    AP_Float _accel_gf;

    // temporary _pitch_max_limit. Cleared on each loop. Clear when >= 90
    int8_t _pitch_max_limit = 90;
    
    // current height estimate (above field elevation)
    float _height;//高度（m）

    // throttle demand in the range from -1.0 to 1.0, usually positive unless reverse thrust is enabled via _THRminf < 0
    float _throttle_dem;//目标油门值（通过_STE_error来计算，PID控制率）

    // pitch angle demand in radians
    float _pitch_dem;//目标俯仰角（受_PITCHminf和_PITCHmaxf限制，最终施加于控制的俯仰角）

    // estimated climb rate (m/s)
    float _climb_rate;//爬升速率（-Vz，m/s）

    // climb and sink rate limits
    float _climb_rate_limit;//最大爬升率
    float _sink_rate_limit;//最大下降率

    /*
      a filter to estimate climb rate if we don't have it from the EKF
     */
    struct {
        // height filter second derivative
        float dd_height;

        // height integration
        float height;
    } _height_filter;//高度滤波器

    // Integrator state 4 - airspeed filter first derivative
    float _integDTAS_state;

    // Integrator state 5 - true airspeed
    float _TAS_state;//真实空速（从空速计获得当量空速，经过转换和二阶互补滤波）

    // Integrator state 6 - throttle integrator
    float _integTHR_state;//油门控制积分因子（_STE_error的积分乘以积分参数）

    // energy balance error integral
    float _integSEBdot;

    // pitch demand kinetic energy error integral
    float _integKE;

    // throttle demand rate limiter state
    float _last_throttle_dem;

    // pitch demand rate limiter state
    float _last_pitch_dem;

    // Rate of change of speed along X axis
    float _vel_dot;//前向加速度（x轴）
    float _vel_dot_lpf;//滤波后的前向加速度（低通滤波）

    // Equivalent airspeed
    float _EAS;

    // True airspeed limits
    float _TASmax;//最大真实空速
    float _TASmin;//最小真实空速

    // Current true airspeed demand
    float _TAS_dem;//目标真实空速（m/s，通过_EAS_dem转换而来）

    // Equivalent airspeed demand
    float _EAS_dem;//目标当量空速（m/s，通过update_pitch_throttle函数传入）

    // height demands
    float _hgt_dem_in_raw;      // height demand input from autopilot before any modification (m) 目标飞行高度（m），来自参数的目标飞行高度
    float _hgt_dem_in;          // height demand input from autopilot after unachievable climb or descent limiting (m) 目标飞行高度（m），来自参数，在有些情况下_hgt_dem_in不会随着_hgt_dem_in_raw更新
    float _hgt_dem_in_prev;     // previous value of _hgt_dem_in (m)
    float _hgt_dem_lpf;         // height demand after application of low pass filtering (m) 目标高度值（_hgt_dem_rate_ltd的低通滤波值）
    float _flare_hgt_dem_adj;   // height rate demand duirng flare adjusted for height tracking offset at flare entry (m)
    float _flare_hgt_dem_ideal; // height we want to fly at during flare (m)
    float _hgt_dem;             // height demand sent to control loops (m) 施加于控制的目标高度值（_hgt_dem_lpf加上起飞偏移量）
    float _hgt_dem_prev;        // _hgt_dem from previous frame (m)

    // height rate demands
    float _hgt_dem_rate_ltd;    // height demand after application of the rate limiter (m) 目标高度值（_hgt_dem_in二值滤波后再取最大爬升/下降率限值）
    float _hgt_rate_dem;        // height rate demand sent to control loops

    // offset applied to height demand post takeoff to compensate for height demand filter lag
    float _post_TO_hgt_offset;

    // last lag compensation offset applied to height demand
    float _lag_comp_hgt_offset;

    // Speed demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_dem_adj;//真实空速目标值（调整后），变化速度不超过STE变化率限制

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_rate_dem;//真实空速变化率目标值
    float _TAS_rate_dem_lpf;

    // Total energy rate filter state
    float _STEdotErrLast;

    // time we started a takeoff
    uint32_t _takeoff_start_ms;

    struct flags {
        // Underspeed condition
        bool underspeed:1;//是否处于失速状态

        // Bad descent condition caused by unachievable airspeed demand
        bool badDescent:1;//是否不良下降

        // true when plane is in auto mode and executing a land mission item
        bool is_doing_auto_land:1;//是否处于自动着陆状态

        // true when we have reached target speed in takeoff
        bool reached_speed_takeoff:1;//是否达到起飞速度

        // true if the soaring feature has requested gliding flight
        bool gliding_requested:1;

        // true when we are in gliding flight, in one of three situations;
        //   - THR_MAX=0
        //   - gliding has been requested e.g. by soaring feature
        //   - engine failure detected (detection not implemented currently)
        bool is_gliding:1;//是否处于滑翔状态

        // true if a propulsion failure is detected.
        bool propulsion_failed:1;

        // true when a reset of airspeed and height states to current is performed on this frame
        bool reset:1;//重启（当更新时间间隔>1.0s时，需要重置控制算法，此标志位置1）
    };
    union {
        struct flags _flags;//标志位
        uint8_t _flags_byte;
    };

    // time when underspeed started
    uint32_t _underspeed_start_ms;

    // auto mode flightstage
    enum AP_FixedWing::FlightStage _flight_stage;//飞行阶段（起飞、正常、降落等）

    // pitch demand before limiting
    float _pitch_dem_unc;//目标俯仰角（根据SEBdot来计算，PID控制率）

    // Maximum and minimum specific total energy rate limits
    float _STEdot_max;//最大STE（Specific Total Energy）变化率
    float _STEdot_min;//最小STE变化率

    // Maximum and minimum floating point throttle limits
    float _THRmaxf;//最大油门
    float _THRminf;//最小油门

    // Maximum and minimum floating point pitch limits
    float _PITCHmaxf;//最大俯仰角
    float _PITCHminf;//最小俯仰角

    // 1 if throttle is clipping at max value, -1 if clipping at min value, 0 otherwise
    enum class clipStatus  : int8_t {
        MIN  = -1,
        NONE =  0,
        MAX  =  1,
    };
    clipStatus _thr_clip_status;

    // Specific energy quantities
    float _SPE_dem;//目标势能（通过目标高度计算）
    float _SKE_dem;//目标动能（通过目标速度计算）
    float _SPEdot_dem;//目标势能变化率
    float _SKEdot_dem;//目标动能变化率
    float _SPE_est;//当前势能（根据高度计算）
    float _SKE_est;//当前动能（根据速度计算）
    float _SPEdot;//势能变化率（通过爬升率计算）
    float _SKEdot;//动能变化率（通过速度和加速度计算）

    // variables used for precision landing pitch control
    float _hgt_at_start_of_flare;
    float _hgt_rate_at_flare_entry;
    float _hgt_afe;//高度估计值（作为update_pitch_throttle参数传入）
    float _pitch_min_at_flare_entry;

    // used to scale max climb and sink limits to match vehicle ability
    float _max_climb_scaler;
    float _max_sink_scaler;

    // Specific energy error quantities
    float _STE_error;//总能量误差

    // 1 when specific energy balance rate demand is clipping in the up direction
    // -1 when specific energy balance rate demand is clipping in the down direction
    // 0 when not clipping
    clipStatus _SEBdot_dem_clip;

    // Time since last update of main TECS loop (seconds)
    float _DT;//TECS控制（update_pitch_throttle）运行时间间隔

    // true when class variables used for flare control have been initialised
    // on flare entry
    bool _flare_initialised;

    // slew height demand lag filter value when transition to land
    float hgt_dem_lag_filter_slew;

    // percent traveled along the previous and next waypoints
    float _path_proportion;

    float _distance_beyond_land_wp;//超过着陆航点的距离

    float _land_pitch_min = -90;//着陆阶段最小俯仰角

    // need to reset on next loop
    bool _need_reset;

    float _SKE_weighting;//动能权重值

    AP_Int8 _use_synthetic_airspeed;
    
    // use synthetic airspeed for next loop
    bool _use_synthetic_airspeed_once;

    // using airspeed in throttle calculation this frame
    bool _using_airspeed_for_throttle;

    // low pass filters used for crossover filter that combines demanded and measured pitch
    // when calculating a pitch to throttle mapping.
    LowPassFilterFloat _pitch_demand_lpf;
    LowPassFilterFloat _pitch_measured_lpf;

    // aerodynamic load factor
    float _load_factor;//载荷因子（额外的重力，比如转弯时）

    // Update the airspeed internal state using a second order complementary filter
    void _update_speed(float DT);

    // Update the demanded airspeed
    void _update_speed_demand(void);

    // Update the demanded height
    void _update_height_demand(void);

    // Detect an underspeed condition
    void _detect_underspeed(void);

    // Update Specific Energy Quantities
    void _update_energies(void);

    // Update Demanded Throttle
    void _update_throttle_with_airspeed(void);

    // Update Demanded Throttle Non-Airspeed
    void _update_throttle_without_airspeed(int16_t throttle_nudge);

    // get integral gain which is flight_stage dependent
    float _get_i_gain(void);

    // Detect Bad Descent
    void _detect_bad_descent(void);

    // Update Demanded Pitch Angle
    void _update_pitch(void);

    // Initialise states and variables
    void _initialise_states(int32_t ptchMinCO_cd, float hgt_afe);

    // Calculate specific total energy rate limits
    void _update_STE_rate_lim(void);

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;//加速度滤波器

    // current time constant
    float timeConstant(void) const;
};

void AP_TECS::update_50hz(void)
{
    // Implement third order complementary filter for height and height rate
    // estimated height rate = _climb_rate
    // estimated height above field elevation  = _height
    // Reference Paper :
    // Optimizing the Gains of the Baro-Inertial Vertical Channel
    // Widnall W.S, Sinha P.K,
    // AIAA Journal of Guidance and Control, 78-1307R

    /*
      if we have a vertical position estimate from the EKF then use
      it, otherwise use barometric altitude
     */
    _ahrs.get_relative_position_D_home(_height);
    _height *= -1.0f;

    // Calculate time in seconds since last update
    uint64_t now = AP_HAL::micros64();
    float DT = (now - _update_50hz_last_usec) * 1.0e-6f;
    _flags.reset = DT > 1.0f;
    if (_flags.reset) {
        _climb_rate = 0.0f;
        _height_filter.dd_height = 0.0f;
        DT = 0.02f; // when first starting TECS, use most likely time constant
        _vdot_filter.reset();
        _takeoff_start_ms = 0;
    }
    _update_50hz_last_usec = now;

    // Use inertial nav verical velocity and height if available
    Vector3f velned;
    if (_ahrs.get_velocity_NED(velned)) {
        // if possible use the EKF vertical velocity
        _climb_rate = -velned.z;
    } else {
        /*
          use a complimentary filter to calculate climb_rate. This is
          designed to minimise lag
         */
        const float baro_alt = AP::baro().get_altitude();
        // Get height acceleration
        float hgt_ddot_mea = -(_ahrs.get_accel_ef().z + GRAVITY_MSS);
        // Perform filter calculation using backwards Euler integration
        // Coefficients selected to place all three filter poles at omega
        float omega2 = _hgtCompFiltOmega*_hgtCompFiltOmega;
        float hgt_err = baro_alt - _height_filter.height;
        float integ1_input = hgt_err * omega2 * _hgtCompFiltOmega;

        _height_filter.dd_height += integ1_input * DT;

        float integ2_input = _height_filter.dd_height + hgt_ddot_mea + hgt_err * omega2 * 3.0f;

        _climb_rate += integ2_input * DT;

        float integ3_input = _climb_rate + hgt_err * _hgtCompFiltOmega * 3.0f;
        // If more than 1 second has elapsed since last update then reset the integrator state
        // to the measured height
        if (_flags.reset) {
            _height_filter.height = _height;
        } else {
            _height_filter.height += integ3_input*DT;
        }
    }

    // Update the speed estimate using a 2nd order complementary filter
    _update_speed(DT);
}

void AP_TECS::_update_pitch(void)
{
    // Calculate Speed/Height Control Weighting
    // This is used to determine how the pitch control prioritises speed and height control
    // A weighting of 1 provides equal priority (this is the normal mode of operation)
    // A SKE_weighting of 0 provides 100% priority to height control. This is used when no airspeed measurement is available
    // A SKE_weighting of 2 provides 100% priority to speed control. This is used when an underspeed condition is detected. In this instance, if airspeed
    // rises above the demanded value, the pitch angle will be increased by the TECS controller.
    _SKE_weighting = constrain_float(_spdWeight, 0.0f, 2.0f);
    if (!(_ahrs.airspeed_sensor_enabled() || _use_synthetic_airspeed)) {
        _SKE_weighting = 0.0f;
    } else if (_flight_stage == AP_FixedWing::FlightStage::VTOL) {
        // if we are in VTOL mode then control pitch without regard to
        // speed. Speed is also taken care of independently of
        // height. This is needed as the usual relationship of speed
        // and height is broken by the VTOL motors
        _SKE_weighting = 0.0f;
    } else if ( _flags.underspeed || _flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING || _flags.is_gliding) {
        _SKE_weighting = 2.0f;
    } else if (_flags.is_doing_auto_land) {
        if (_spdWeightLand < 0) {
            // use sliding scale from normal weight down to zero at landing
            float scaled_weight = _spdWeight * (1.0f - constrain_float(_path_proportion,0,1));
            _SKE_weighting = constrain_float(scaled_weight, 0.0f, 2.0f);
        } else {
            _SKE_weighting = constrain_float(_spdWeightLand, 0.0f, 2.0f);
        }
    }

    float SPE_weighting = 2.0f - _SKE_weighting;

    // either weight can fade to 0, but don't go above 1 to prevent instability if tuned at a speed weight of 1 and wieghting is varied to end points in flight.
    SPE_weighting = MIN(SPE_weighting, 1.0f);
    _SKE_weighting = MIN(_SKE_weighting, 1.0f);

    // Calculate demanded specific energy balance and error
    float SEB_dem = _SPE_dem * SPE_weighting - _SKE_dem * _SKE_weighting;
    float SEB_est = _SPE_est * SPE_weighting - _SKE_est * _SKE_weighting;
    float SEB_error = SEB_dem - SEB_est;

    // track demanded height using the specified time constant
    float SEBdot_dem = _hgt_rate_dem * GRAVITY_MSS * SPE_weighting + SEB_error / timeConstant();
    const float SEBdot_dem_min = - _maxSinkRate * GRAVITY_MSS;
    const float SEBdot_dem_max = _maxClimbRate * GRAVITY_MSS;
    if (SEBdot_dem < SEBdot_dem_min) {
        SEBdot_dem = SEBdot_dem_min;
        _SEBdot_dem_clip = clipStatus::MIN;
    } else if (SEBdot_dem > SEBdot_dem_max) {
        SEBdot_dem = SEBdot_dem_max;
        _SEBdot_dem_clip = clipStatus::MAX;
    } else {
        _SEBdot_dem_clip = clipStatus::NONE;
    }

    // calculate specific energy balance rate error
    const float SEBdot_est = _SPEdot * SPE_weighting - _SKEdot * _SKE_weighting;
    float SEBdot_error = SEBdot_dem - SEBdot_est;

    // sum predicted plus damping correction
    // integral correction is added later
    // During flare a different damping gain is used
    float pitch_damp = _ptchDamp;
    if (_landing.is_flaring()) {
        pitch_damp = _landDamp;
    } else if (!is_zero(_land_pitch_damp) && _flags.is_doing_auto_land) {
        pitch_damp = _land_pitch_damp;
    }
    float SEBdot_dem_total = SEBdot_dem + SEBdot_error * pitch_damp;

    // inverse of gain from SEB to pitch angle
    float gainInv = (_TAS_state * GRAVITY_MSS);

    // During climbout/takeoff, bias the demanded pitch angle so that zero speed error produces a pitch angle
    // demand equal to the minimum value (which is )set by the mission plan during this mode). Otherwise the
    // integrator has to catch up before the nose can be raised to reduce speed during climbout.
    if (_flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        SEBdot_dem_total += _PITCHminf * gainInv;
    }

    // don't allow the integrator to rise by more than 20% of its full
    // Calculate max and min values for integrator state that will allow for no more than
    // 5deg of saturation. This allows for some pitch variation due to gusts before the
    // integrator is clipped. Otherwise the effectiveness of the integrator will be reduced in turbulence
    float integSEBdot_min = (gainInv * (_PITCHminf - radians(5.0f))) - SEBdot_dem_total;
    float integSEBdot_max = (gainInv * (_PITCHmaxf + radians(5.0f))) - SEBdot_dem_total;

    // Calculate integrator state, constraining input if pitch limits are exceeded
    // don't allow the integrator to rise by more than 10% of its full
    // range in one step. This prevents single value glitches from
    // causing massive integrator changes. See Issue#4066
    float integSEB_range = integSEBdot_max - integSEBdot_min;
    float integSEB_delta = constrain_float(SEBdot_error * _get_i_gain() * _DT, -integSEB_range*0.1f, integSEB_range*0.1f);

    // predict what pitch will be with uncontrained integration
    _pitch_dem_unc = (SEBdot_dem_total + _integSEBdot + integSEB_delta + _integKE) / gainInv;

    // integrate SEB rate error and apply integrator state limits
    const bool inhibit_integrator = ((_pitch_dem_unc > _PITCHmaxf) && integSEB_delta > 0.0f) ||
                                    ((_pitch_dem_unc < _PITCHminf) && integSEB_delta < 0.0f);
    if (!inhibit_integrator) {
        _integSEBdot += integSEB_delta;
        _integKE += (_SKE_est - _SKE_dem) * _SKE_weighting * _DT / timeConstant();
    } else {
        // fade out integrator if saturating
        const float coef = 1.0f - _DT / (_DT + timeConstant());
        _integSEBdot *= coef;
        _integKE *= coef;
    }
    _integSEBdot = constrain_float(_integSEBdot, integSEBdot_min, integSEBdot_max);
    const float KE_integ_limit = 0.25f * (_PITCHmaxf - _PITCHminf) * gainInv; // allow speed trim integrator to access 505 of pitch range
    _integKE = constrain_float(_integKE, - KE_integ_limit, KE_integ_limit);

    // Calculate pitch demand from specific energy balance signals
    _pitch_dem_unc = (SEBdot_dem_total + _integSEBdot + _integKE) / gainInv;

    // Add a feedforward term from demanded airspeed to pitch
    if (_flags.is_gliding) {
        _pitch_dem_unc += (_TAS_dem_adj - _pitch_ff_v0) * _pitch_ff_k;
    }

    // Constrain pitch demand
    _pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);

    // Rate limit the pitch demand to comply with specified vertical
    // acceleration limit
    float ptchRateIncr = _DT * _vertAccLim / _TAS_state;

    if ((_pitch_dem - _last_pitch_dem) > ptchRateIncr) {
        _pitch_dem = _last_pitch_dem + ptchRateIncr;
    } else if ((_pitch_dem - _last_pitch_dem) < -ptchRateIncr) {
        _pitch_dem = _last_pitch_dem - ptchRateIncr;
    }

    _last_pitch_dem = _pitch_dem;

    if (AP::logger().should_log(_log_bitmask)){
        AP::logger().WriteStreaming("TEC2","TimeUS,PEW,EBD,EBE,EBDD,EBDE,EBDDT,Imin,Imax,I,KI,pmin,pmax",
                                    "Qffffffffffff",
                                    AP_HAL::micros64(),
                                    (double)SPE_weighting,
                                    (double)SEB_dem,
                                    (double)SEB_est,
                                    (double)SEBdot_dem,
                                    (double)SEBdot_est,
                                    (double)SEBdot_dem_total,
                                    (double)integSEBdot_min,
                                    (double)integSEBdot_max,
                                    (double)_integSEBdot,
                                    (double)_integKE,
                                    (double)_PITCHminf,
                                    (double)_PITCHmaxf);
    }
}

/*
  calculate throttle demand - airspeed enabled case
 */
void AP_TECS::_update_throttle_with_airspeed(void)
{
    // Calculate limits to be applied to potential energy error to prevent over or underspeed occurring due to large height errors
    float SPE_err_max = MAX(_SKE_est - 0.5f * _TASmin * _TASmin, 0.0f);
    float SPE_err_min = MIN(_SKE_est - 0.5f * _TASmax * _TASmax, 0.0f);

    if (_flight_stage == AP_FixedWing::FlightStage::VTOL) {
        /*
          when we are in a VTOL state then we ignore potential energy
          errors as we have vertical motors that interfere with the
          total energy calculation.
         */
        SPE_err_max = SPE_err_min = 0;
    }

    // rate of change of potential energy is proportional to height error
    _SPEdot_dem = (_SPE_dem - _SPE_est) / timeConstant();

    // Calculate total energy error
    _STE_error = constrain_float((_SPE_dem - _SPE_est), SPE_err_min, SPE_err_max) + _SKE_dem - _SKE_est;
    float STEdot_dem = constrain_float((_SPEdot_dem + _SKEdot_dem), _STEdot_min, _STEdot_max);
    float STEdot_error = STEdot_dem - _SPEdot - _SKEdot;

    // Apply 0.5 second first order filter to STEdot_error
    // This is required to remove accelerometer noise from the  measurement
    const float filt_coef = 2.0f * _DT;
    STEdot_error = filt_coef * STEdot_error + (1.0f - filt_coef) * _STEdotErrLast;
    _STEdotErrLast = STEdot_error;

    // Calculate throttle demand
    // If underspeed condition is set, then demand full throttle
    if (_flags.underspeed) {
        _throttle_dem = 1.0f;
    } else if (_flags.is_gliding) {
        _throttle_dem = 0.0f;
    } else {
        // Calculate gain scaler from specific energy error to throttle
        // (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf) is the derivative of STEdot wrt throttle measured across the max allowed throttle range.
        const float K_STE2Thr = 1 / (timeConstant() * (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf));

        // Calculate feed-forward throttle
        const float nomThr = aparm.throttle_cruise * 0.01f;
        const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
        // Use the demanded rate of change of total energy as the feed-forward demand, but add
        // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
        // drag increase during turns.
        const float cosPhi = sqrtf((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
        STEdot_dem = STEdot_dem + _rollComp * (1.0f/constrain_float(cosPhi * cosPhi, 0.1f, 1.0f) - 1.0f);
        const float ff_throttle = nomThr + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);

        // Calculate PD + FF throttle
        float throttle_damp = _thrDamp;
        if (_flags.is_doing_auto_land && !is_zero(_land_throttle_damp)) {
            throttle_damp = _land_throttle_damp;
        }
        _throttle_dem = (_STE_error + STEdot_error * throttle_damp) * K_STE2Thr + ff_throttle;

        float THRminf_clipped_to_zero = constrain_float(_THRminf, 0, _THRmaxf);

        // Calculate integrator state upper and lower limits
        // Set to a value that will allow 0.1 (10%) throttle saturation to allow for noise on the demand
        // Additionally constrain the integrator state amplitude so that the integrator comes off limits faster.
        const float maxAmp = 0.5f*(_THRmaxf - THRminf_clipped_to_zero);
        const float integ_max = constrain_float((_THRmaxf - _throttle_dem + 0.1f),-maxAmp,maxAmp);
        const float integ_min = constrain_float((_THRminf - _throttle_dem - 0.1f),-maxAmp,maxAmp);

        // Calculate integrator state, constraining state
        // Set integrator to a max throttle value during climbout
        _integTHR_state = _integTHR_state + (_STE_error * _get_i_gain()) * _DT * K_STE2Thr;
        if (_flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
            if (!_flags.reached_speed_takeoff) {
                // ensure we run at full throttle until we reach the target airspeed
                _throttle_dem = MAX(_throttle_dem, _THRmaxf - _integTHR_state);
            }
            _integTHR_state = integ_max;
        } else {
            _integTHR_state = constrain_float(_integTHR_state, integ_min, integ_max);
        }

        // Rate limit PD + FF throttle
        // Calculate the throttle increment from the specified slew time
        int8_t throttle_slewrate = aparm.throttle_slewrate;
        if (_landing.is_on_approach()) {
            const int8_t land_slewrate = _landing.get_throttle_slewrate();
            if (land_slewrate > 0) {
                throttle_slewrate = land_slewrate;
            }
        }

        if (throttle_slewrate != 0) {
            const float thrRateIncr = _DT * (_THRmaxf - THRminf_clipped_to_zero) * throttle_slewrate * 0.01f;

            _throttle_dem = constrain_float(_throttle_dem,
                                            _last_throttle_dem - thrRateIncr,
                                            _last_throttle_dem + thrRateIncr);
            _last_throttle_dem = _throttle_dem;
        }

        // Sum the components.
        _throttle_dem = _throttle_dem + _integTHR_state;

        if (AP::logger().should_log(_log_bitmask)){
            AP::logger().WriteStreaming("TEC3","TimeUS,KED,PED,KEDD,PEDD,TEE,TEDE,FFT,Imin,Imax,I,Emin,Emax",
                                        "Qffffffffffff",
                                        AP_HAL::micros64(),
                                        (double)_SKEdot,
                                        (double)_SPEdot,
                                        (double)_SKEdot_dem,
                                        (double)_SPEdot_dem,
                                        (double)_STE_error,
                                        (double)STEdot_error,
                                        (double)ff_throttle,
                                        (double)integ_min,
                                        (double)integ_max,
                                        (double)_integTHR_state,
                                        (double)SPE_err_min,
                                        (double)SPE_err_max);
        }
    }

    // Constrain throttle demand and record clipping
    if (_throttle_dem > _THRmaxf) {
        _thr_clip_status = clipStatus::MAX;
        _throttle_dem = _THRmaxf;
    } else if (_throttle_dem < _THRminf) {
        _thr_clip_status = clipStatus::MIN;
        _throttle_dem = _THRminf;
    } else {
        _thr_clip_status = clipStatus::NONE;
    }
}

void AP_TECS::update_pitch_throttle(int32_t hgt_dem_cm,//目标高度，通常指相对于起飞点的高度，单位厘米
                                    int32_t EAS_dem_cm,//目标当量速度，单位厘米/秒
                                    enum AP_FixedWing::FlightStage// flight_stage,飞行阶段
                                    float distance_beyond_land_wp,//超过着陆航点距离
                                    int32_t ptchMinCO_cd,//最小俯仰角，单位centi-degree（1/100度）
                                    int16_t throttle_nudge,//油门微调，范围通常为（-100,100）
                                    float hgt_afe,//实际高度，相对于起飞点，单位米
                                    float load_factor)//载荷因子，单位g（重力加速度），用以补偿转弯时的能量损失
{
    uint64_t now = AP_HAL::micros64();
    // check how long since we last did the 50Hz update; do nothing in
    // this loop if that hasn't run for some signficant period of
    // time.  Notably, it may never have run, leaving _TAS_state as
    // zero and subsequently division-by-zero errors.
    const float _DT_for_update_50hz = (now - _update_50hz_last_usec) * 1.0e-6f;
    if (_update_50hz_last_usec == 0 || _DT_for_update_50hz > 1.0) {
        // more than 1 second since it was run, don't do anything yet:
        return;
    }

    // Calculate time in seconds since last update
    _DT = (now - _update_pitch_throttle_last_usec) * 1.0e-6f;
    _DT = MAX(_DT, 0.001f);
    _update_pitch_throttle_last_usec = now;

    _flags.is_gliding = _flags.gliding_requested || _flags.propulsion_failed || aparm.throttle_max==0;
    _flags.is_doing_auto_land = (flight_stage == AP_FixedWing::FlightStage::LAND);
    _distance_beyond_land_wp = distance_beyond_land_wp;
    _flight_stage = flight_stage;

    // Convert inputs
    _hgt_dem_in_raw = hgt_dem_cm * 0.01f;
    _EAS_dem = EAS_dem_cm * 0.01f;
    _hgt_afe = hgt_afe;
    _load_factor = load_factor;

    // Don't allow height deamnd to continue changing in a direction that saturates vehicle manoeuvre limits
    // if vehicle is unable to follow the demanded climb or descent.
    const bool max_climb_condition = (_pitch_dem_unc > _PITCHmaxf || _thr_clip_status == clipStatus::MAX) &&
                                    !(_flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING);
    const bool max_descent_condition = _pitch_dem_unc < _PITCHminf || _thr_clip_status == clipStatus::MIN;
    if (max_climb_condition && _hgt_dem_in_raw > _hgt_dem_in_prev) {
        _hgt_dem_in = _hgt_dem_in_prev;
    } else if (max_descent_condition && _hgt_dem_in_raw < _hgt_dem_in_prev) {
        _hgt_dem_in = _hgt_dem_in_prev;
    } else {
        _hgt_dem_in = _hgt_dem_in_raw;
    }

    if (aparm.takeoff_throttle_max != 0 &&
        (_flight_stage == AP_FixedWing::FlightStage::TAKEOFF || _flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING)) {
        _THRmaxf  = aparm.takeoff_throttle_max * 0.01f;
    } else {
        _THRmaxf  = aparm.throttle_max * 0.01f;
    }
    _THRminf  = aparm.throttle_min * 0.01f;

    // min of 1% throttle range to prevent a numerical error
    _THRmaxf = MAX(_THRmaxf, _THRminf+0.01);

    // work out the maximum and minimum pitch
    // if TECS_PITCH_{MAX,MIN} isn't set then use
    // LIM_PITCH_{MAX,MIN}. Don't allow TECS_PITCH_{MAX,MIN} to be
    // larger than LIM_PITCH_{MAX,MIN}
    if (_pitch_max == 0) {
        _PITCHmaxf = aparm.pitch_limit_max_cd * 0.01f;
    } else {
        _PITCHmaxf = MIN(_pitch_max, aparm.pitch_limit_max_cd * 0.01f);
    }

    if (_pitch_min >= 0) {
        _PITCHminf = aparm.pitch_limit_min_cd * 0.01f;
    } else {
        _PITCHminf = MAX(_pitch_min, aparm.pitch_limit_min_cd * 0.01f);
    }

    // apply temporary pitch limit and clear
    if (_pitch_max_limit < 90) {
        _PITCHmaxf = constrain_float(_PITCHmaxf, -90, _pitch_max_limit);
        _PITCHminf = constrain_float(_PITCHminf, -_pitch_max_limit, _PITCHmaxf);
        _pitch_max_limit = 90;
    }

    if (!_landing.is_on_approach()) {
        // reset land pitch min when not landing
        _land_pitch_min = _PITCHminf;
    }

    // calculate the expected pitch angle from the demanded climb rate and airspeed fo ruse during approach and flare
    if (_landing.is_flaring()) {
        // smoothly move the min pitch to the required minimum at touchdown
        float p; // 0 at start of flare, 1 at finish
        if (!_flare_initialised) {
            p = 0.0f;
        } else if (_hgt_at_start_of_flare > _flare_holdoff_hgt) {
            p = constrain_float((_hgt_at_start_of_flare - _hgt_afe) / _hgt_at_start_of_flare, 0.0f, 1.0f);
        } else {
            p = 1.0f;
        }
        const float pitch_limit_deg = (1.0f - p) * _pitch_min_at_flare_entry + p * 0.01f * _landing.get_pitch_cd();

        // in flare use min pitch from LAND_PITCH_CD
        _PITCHminf = MAX(_PITCHminf, pitch_limit_deg);

        // and use max pitch from TECS_LAND_PMAX
        if (_land_pitch_max != 0) {
            // note that this allows a flare pitch outside the normal TECS auto limits
            _PITCHmaxf = _land_pitch_max;
        }

        // and allow zero throttle
        _THRminf = 0;
    } else if (_landing.is_on_approach()) {
        _PITCHminf = MAX(_PITCHminf, 0.01f * aparm.pitch_limit_min_cd);
        _pitch_min_at_flare_entry = _PITCHminf;
        _flare_initialised = false;
    } else {
        _flare_initialised = false;
    }

    if (_landing.is_on_approach()) {
        // don't allow the lower bound of pitch to decrease, nor allow
        // it to increase rapidly. This prevents oscillation of pitch
        // demand while in landing approach based on rapidly changing
        // time to flare estimate
        if (_land_pitch_min <= -90) {
            _land_pitch_min = _PITCHminf;
        }
        const float flare_pitch_range = 20;
        const float delta_per_loop = (flare_pitch_range/_landTimeConst) * _DT;
        _PITCHminf = MIN(_PITCHminf, _land_pitch_min+delta_per_loop);
        _land_pitch_min = MAX(_land_pitch_min, _PITCHminf);
        _PITCHminf = MAX(_land_pitch_min, _PITCHminf);
    }

    if (flight_stage == AP_FixedWing::FlightStage::TAKEOFF || flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        if (!_flags.reached_speed_takeoff && _TAS_state >= _TAS_dem_adj) {
            // we have reached our target speed in takeoff, allow for
            // normal throttle control
            _flags.reached_speed_takeoff = true;
        }
    }

    // convert to radians
    _PITCHmaxf = radians(_PITCHmaxf);
    _PITCHminf = radians(_PITCHminf);

    // don't allow max pitch to go below min pitch
    _PITCHmaxf = MAX(_PITCHmaxf, _PITCHminf);

    // initialise selected states and variables if DT > 1 second or in climbout
    _initialise_states(ptchMinCO_cd, hgt_afe);

    // Calculate Specific Total Energy Rate Limits
    _update_STE_rate_lim();

    // Calculate the speed demand
    _update_speed_demand();

    // Calculate the height demand
    _update_height_demand();

    // Detect underspeed condition
    _detect_underspeed();

    // Calculate specific energy quantitiues
    _update_energies();

    // Calculate pitch demand
    _update_pitch();

    // Calculate throttle demand - use simple pitch to throttle if no
    // airspeed sensor.
    // Note that caller can demand the use of
    // synthetic airspeed for one loop if needed. This is required
    // during QuadPlane transition when pitch is constrained
    if (_ahrs.airspeed_sensor_enabled() || _use_synthetic_airspeed || _use_synthetic_airspeed_once) {
        _update_throttle_with_airspeed();
        _use_synthetic_airspeed_once = false;
        _using_airspeed_for_throttle = true;
    } else {
        _update_throttle_without_airspeed(throttle_nudge);
        _using_airspeed_for_throttle = false;
    }

    // Detect bad descent due to demanded airspeed being too high
    _detect_bad_descent();

    if (_options & OPTION_GLIDER_ONLY) {
        _flags.badDescent = false;
    }

    if (AP::logger().should_log(_log_bitmask)){
        // log to AP_Logger
        // @LoggerMessage: TECS
        // @Vehicles: Plane
        // @Description: Information about the Total Energy Control System
        // @URL: http://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html
        // @Field: TimeUS: Time since system startup
        // @Field: h: height estimate (UP) currently in use by TECS
        // @Field: dh: current climb rate ("delta-height")
        // @Field: hin: height demand received by TECS
        // @Field: hdem: height demand after rate limiting and filtering that TECS is currently trying to achieve
        // @Field: dhdem: climb rate TECS is currently trying to achieve
        // @Field: spdem: True AirSpeed TECS is currently trying to achieve
        // @Field: sp: current estimated True AirSpeed
        // @Field: dsp: x-axis acceleration estimate ("delta-speed")
        // @Field: th: throttle output
        // @Field: ph: pitch output
        // @Field: pmin: pitch lower limit
        // @Field: pmax: pitch upper limit
        // @Field: dspdem: demanded acceleration output ("delta-speed demand")
        // @Field: f: flags
        // @FieldBits: f: Underspeed,UnachievableDescent,AutoLanding,ReachedTakeoffSpd
        AP::logger().WriteStreaming("TECS", "TimeUS,h,dh,hin,hdem,dhdem,spdem,sp,dsp,th,ph,pmin,pmax,dspdem,f",
                                    "smnmmnnnn------",
                                    "F00000000------",
                                    "QfffffffffffffB",
                                    now,
                                    (double)_height,
                                    (double)_climb_rate,
                                    (double)_hgt_dem_in_raw,
                                    (double)_hgt_dem,
                                    (double)_hgt_rate_dem,
                                    (double)_TAS_dem_adj,
                                    (double)_TAS_state,
                                    (double)_vel_dot,
                                    (double)_throttle_dem,
                                    (double)_pitch_dem,
                                    (double)_PITCHminf,
                                    (double)_PITCHmaxf,
                                    (double)_TAS_rate_dem,
                                    _flags_byte);
    }
}
```

TECS包（在update_pitch_throttle中调用记录）
|字段|说明|
| :-----: | :-----: |
|TimeUS|时间（us）|
|h|高度（_height）|
|dh|爬升率（_climb_rate）|
|hin|目标高度（参数输入，_hgt_dem_in_raw）|
|hdem|目标高度（实际控制目标高度，_hgt_dem）|
|dhdem|目标爬升率（_hgt_rate_dem）|
|spdem|目标飞行速度（_TAS_dem_adj）|
|sp|真实速度（_TAS_state）|
|dsp|加速度（前向，_vel_dot）|
|th|目标油门值（_throttle_dem）|
|ph|目标俯仰角（_pitch_dem）|
|pmin|最小俯仰角（_PITCHminf）|
|pmax|最大俯仰角（_PITCHmaxf）|
|dspdem|目标加速度（_TAS_rate_dem）|
|f|标志位（_flags_byte，和_flags是union）|

TEC2包（在_update_pitch中调用记录）
|字段|说明|
| :-----: | :-----: |
|TimeUS|时间（us）|
|PEW|SPE_weighting，势能权重值|
|KEW|_SKE_weighting，动能权重值|
|EBD|SEB_dem，目标平衡能量|
|EBE|SEB_est，估计平衡能量值|
|EBDD|SEBdot_dem，目标平衡能量变化率|
|EBDE|SEBdot_est，估计平衡能量变化率|
|EBDDT|SEBdot_dem_total，目标平衡能量变化率（总体）|
|Imin|integSEBdot_min，平衡能量变化率误差积分下限|
|Imax|integSEBdot_max，平衡能量变化率误差积分上限|
|I|_integSEBdot，平衡能量变化率误差积分|
|KI|_integKE，动能误差积分（integral of SKE）|
|tmin|_THRminf，最小油门值|
|tmax|_THRmaxf，最大油门值|

TEC3包（在_update_throttle_with_airspeed中调用记录）
|字段|说明|
| :-----: | :-----: |
|TimeUS|时间（us）|
|KED|_SKEdot，动能变化率|
|PED|_SPEdot，势能变化率|
|KEDD|_SKEdot_dem，目标动能变化率|
|PEDD|_SPEdot_dem，目标势能变化率|
|TEE|_STE_error，总能量误差|
|TEDE|STEdot_error，总能量误差变化率|
|FFT|ff_throttle，前馈油门控制|
|Imin|integ_min，油门控制积分下限|
|Imax|integ_max，油门控制积分上限|
|I|_integTHR_state，油门控制积分量|
|Emin|SPE_err_min，势能误差最小值|
|Emax|SPE_err_max，势能误差最大值|

## 算法

### 系统辨识

### 控制器设计
为什么不基于状态空间进行设计？