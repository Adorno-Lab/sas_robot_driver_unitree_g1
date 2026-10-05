#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <marinholab/sas/core/sas_shutdown_signaler.hpp>
#include <unitree_drivers/DriverUnitreeLocoClient.h>
#include <unitree_drivers/DriverUnitreeArmSDK.h>
#include <unitree_drivers/DriverUnitreeLowState.h>

/**
 * @brief G1 driver: aggregates DriverUnitreeLocoClient (configured for ROBOT::G1) for
 *        high-level locomotion, DriverUnitreeG1ArmSDK for rt/arm_sdk arm/waist
 *        control, and DriverUnitreeLowState (configured for ROBOT::G1) for read-only
 *        rt/lowstate telemetry (every joint's measured position/velocity/torque, plus
 *        the onboard IMU). No longer owns any Unitree SDK type directly -- all three
 *        sub-controllers each hide their own SDK details behind their own pImpl, so
 *        this class doesn't need one of its own.
 *
 * @details All joint-position getters and setters exposed here (arms and waist) use
 *          Eigen::VectorXd rather than fixed-size std::array, giving callers a single,
 *          uniform vector-based interface regardless of which joint group they're
 *          addressing (7 DoF per arm, 3 DoF for the waist). Internally,
 *          DriverUnitreeG1 converts between Eigen::VectorXd and the fixed-size
 *          std::array types used by DriverUnitreeG1ArmSDK -- see that class directly
 *          if you need its native array-based interface instead (e.g. for
 *          compile-time size checking). low_state_'s full-body joint readings
 *          (get_all_joint_positions() and friends) are exposed the same way, as
 *          Eigen::VectorXd, even though low_state_'s own native interface returns
 *          std::vector<double> (its joint count varies by robot/message set, so it
 *          can't use a fixed-size std::array in the first place).
 *
 *          Non-joint-position data -- FSM/balance-mode integers, swing/stand height,
 *          gait phase, high-level velocity targets, and the IMU reading -- keeps its
 *          original type on this class; the VectorXd requirement applies specifically
 *          to joint positions.
 *
 * @details OPERATION_MODE::HIGH_LEVEL vs OPERATION_MODE::LOW_LEVEL: fixed for the
 *          instance's whole lifetime by the constructor (there is no runtime switch).
 *          - HIGH_LEVEL: connect()/initialize()/deinitialize()/disconnect() drive all
 *            three sub-controllers (loco_client_, arm_sdk_, low_state_), and every
 *            method that delegates to loco_client_ or arm_sdk_ (set_mode(), the
 *            FSM/balance/height/velocity getters and setters, the arm-control
 *            enable/disable/query methods, and the arm/waist joint-position setters
 *            and getters) is available.
 *          - LOW_LEVEL: connect()/initialize()/deinitialize()/disconnect() only touch
 *            low_state_ -- loco_client_ and arm_sdk_ are never connected/initialized,
 *            so calling any of the loco_client_/arm_sdk_-delegating methods above
 *            throws std::logic_error. Low-level joint *commands* are meant to go
 *            through a DriverUnitreeLowCmd member, but that class isn't available
 *            yet, so the joint-position setters (which would otherwise have no path
 *            to the robot in this mode) also throw std::logic_error in LOW_LEVEL,
 *            rather than silently doing nothing.
 *          In both modes, low_state_'s read-only telemetry (get_num_joints(),
 *          get_all_joint_positions()/velocities()/torques(), get_imu_data(), and the
 *          per-limb telemetry getters) stays available -- it never depends on
 *          operation_mode_.
 *
 * @throws std::invalid_argument Every joint-position setter validates that the given
 *         Eigen::VectorXd has the exact size expected for that joint group (7 for an
 *         arm, 3 for the waist) and throws std::invalid_argument otherwise, before
 *         forwarding to arm_sdk_. This check happens before the operation-mode check,
 *         so a wrongly-sized vector is reported as a size error even in LOW_LEVEL.
 *
 * @throws std::logic_error Any HIGH_LEVEL-only method (see above) throws
 *         std::logic_error if called while operation_mode_ == OPERATION_MODE::LOW_LEVEL.
 *         In LOW_LEVEL, the joint-position setters throw std::logic_error for a
 *         different reason -- there is currently no low-level command path
 *         (DriverUnitreeLowCmd isn't available yet) -- see throw_low_level_unsupported().
 *
 * @warning Per DriverUnitreeG1ArmSDK's documented behavior: engaging arm control
 *          (weight -> 1.0) suppresses the robot's response to
 *          set_target_high_level_velocities() while in Running/walking mode, and it
 *          resumes responding once arm control is disengaged (weight -> 0.0). These two
 *          sub-controllers are not truly independent even though they publish to
 *          different topics -- coordinate enabling/disabling arm control with whatever
 *          is driving locomotion in your application code. set_mode() below automates
 *          exactly that coordination (see its own docs for the exact sequence), so
 *          application code should generally prefer it over calling
 *          enable_arm_control()/disable_arm_control()/set_balance_mode() by hand.
 *          (Applies to HIGH_LEVEL only.)
 */
class DriverUnitreeG1
{
public:
    /**
     * @brief The two mutually-exclusive high-level operating modes coordinated by
     *        set_mode(). See set_mode() for exactly what switching between them does.
     *        Only meaningful in OPERATION_MODE::HIGH_LEVEL.
     */
    enum class HIGH_MODE
    {
        LOCOMOTION,  ///< Velocity commands (set_target_high_level_velocities()) drive the robot.
        ARM_CONTROL  ///< rt/arm_sdk drives the arms/waist; velocity commands are suppressed while engaged.
    };
    /**
     * @brief Which control path this instance uses; fixed for the instance's whole
     *        lifetime by the constructor. See the class-level @details for exactly
     *        what each mode does and does not allow.
     */
    enum class OPERATION_MODE
    {
        LOW_LEVEL,
        HIGH_LEVEL
    };
    const OPERATION_MODE operation_mode_;

private:
    int32_t domain_id_{0};                ///< DDS domain ID shared by loco_client_, arm_sdk_, and low_state_.
    std::string network_interface_{"lo"}; ///< Network interface name for the DDS channel factory.

    DriverUnitreeLocoClient loco_client_; ///< High-level locomotion; constructed with DriverUnitreeLocoClient::ROBOT::G1. Only connected/initialized/used in OPERATION_MODE::HIGH_LEVEL.
    DriverUnitreeArmSDK arm_sdk_;       ///< rt/arm_sdk arm + waist control. Only connected/initialized/used in OPERATION_MODE::HIGH_LEVEL.
    DriverUnitreeLowState low_state_;     ///< Read-only rt/lowstate telemetry; constructed with DriverUnitreeLowState::ROBOT::G1. Always connected/initialized/used, in both operation modes.
    //DriverUnitreeLowCmd low_cmd_;       ///< To command the robot joints in LOW_LEVEL. Not yet available -- see throw_low_level_unsupported().

    /**
     * @brief Validates that a joint-position vector has the expected size.
     * @param vector The vector to validate.
     * @param expected_size The required size for the joint group being set (7 for an arm, 3 for the waist).
     * @param method_name Name of the calling setter, used in the exception message.
     * @throws std::invalid_argument if vector.size() != expected_size.
     */
    static void _validate_size(const Eigen::VectorXd& vector,
                              const Eigen::Index& expected_size,
                              const std::string& method_name);

    /**
     * @brief Guards a method that only makes sense in OPERATION_MODE::HIGH_LEVEL
     *        (i.e. one that delegates to loco_client_ or arm_sdk_).
     * @param method_name Name of the calling method, used in the exception message.
     * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
     */
    void _ensure_high_level(const std::string& method_name) const;

    /**
     * @brief Called by the joint-position setters when operation_mode_ ==
     *        OPERATION_MODE::LOW_LEVEL. Low-level joint commands are meant to go
     *        through a DriverUnitreeLowCmd member, which isn't available yet, so
     *        there is currently no way to honor these setters in LOW_LEVEL -- this
     *        throws rather than silently doing nothing or falling back to arm_sdk_
     *        (which isn't connected/initialized in this mode).
     * @param method_name Name of the calling setter, used in the exception message.
     * @throws std::logic_error unconditionally.
     */
    [[noreturn]] void _throw_low_level_unsupported(const std::string& method_name) const;

public:
    ~DriverUnitreeG1();
    DriverUnitreeG1()=delete;
    // Delete copy constructor and assignment (prevents double initialization)
    DriverUnitreeG1(const DriverUnitreeG1&) = delete;
    DriverUnitreeG1& operator=(const DriverUnitreeG1&) = delete;
    DriverUnitreeG1(DriverUnitreeG1&&) = delete;
    DriverUnitreeG1& operator=(DriverUnitreeG1&&) = delete;

    /**
     * @brief Constructs the aggregated G1 driver. No hardware/DDS I/O happens here;
     *        see connect().
     * @param shutdown_signaler Shared sas::ShutdownSignaler (typically the same one a
     *        SIGINT handler calls shutdown() on), forwarded as-is to loco_client_,
     *        arm_sdk_, and low_state_ so all three sub-drivers poll the same
     *        should_shutdown() state.
     * @param op_mode Which control path this instance uses for its entire lifetime
     *        (see the class-level @details). Stored in operation_mode_, which is
     *        const -- there is no runtime switch between modes.
     * @param domain_id DDS domain ID used to initialize the process-wide
     *        unitree::robot::ChannelFactory in connect(), and shared by loco_client_,
     *        arm_sdk_, and low_state_.
     * @param network_interface Network interface name (e.g. "enp6s0") for the DDS
     *        channel factory.
     * @throws std::invalid_argument if shutdown_signaler is nullptr. Propagates from
     *         loco_client_'s, arm_sdk_'s, or low_state_'s own constructor, all of
     *         which validate this before DriverUnitreeG1's own constructor body runs.
     */
    explicit DriverUnitreeG1(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler,
                             const OPERATION_MODE& op_mode,
                             const int32_t& domain_id=0,
                             const std::string& network_interface="enp6s0");

    // --- Required methods for SAS ---

    /**
     * @brief Initializes the process-wide DDS channel factory (a shared precondition
     *        for loco_client_, arm_sdk_, and low_state_), then connects the
     *        sub-drivers relevant to operation_mode_.
     * @details HIGH_LEVEL: connects loco_client_, arm_sdk_, then low_state_.
     *          LOW_LEVEL: connects only low_state_ (loco_client_ and arm_sdk_ are
     *          left untouched -- see the class-level @details).
     */
    void connect();

    /**
     * @brief Initializes the sub-drivers relevant to operation_mode_.
     * @details HIGH_LEVEL: initializes loco_client_, arm_sdk_, then low_state_.
     *          LOW_LEVEL: initializes only low_state_.
     */
    void initialize();

    /**
     * @brief Deinitializes the sub-drivers relevant to operation_mode_.
     * @details HIGH_LEVEL: arm_sdk_ (ramp arm-control weight to 0, holding the last
     *          pose), then loco_client_, then low_state_ (a no-op beyond flipping a
     *          flag; see its class docs).
     *          LOW_LEVEL: only low_state_.
     */
    void deinitialize();

    /**
     * @brief Disconnects the sub-drivers relevant to operation_mode_.
     * @details HIGH_LEVEL: arm_sdk_, loco_client_, then low_state_.
     *          LOW_LEVEL: only low_state_.
     */
    void disconnect();

    /**
     * @brief Switches between LOCOMOTION and ARM_CONTROL, performing whatever
     *        hand-off is required so the robot actually responds afterward, instead
     *        of leaving the caller to sequence enable/disable_arm_control(),
     *        set_balance_mode(), and a wait between them by hand (which is exactly
     *        the fixed-sleep-guessing-a-ramp-duration pattern this method replaces).
     *
     * @details
     *  - MODE::ARM_CONTROL: calls arm_sdk_.enable_arm_control(). Non-blocking -- the
     *    blend weight ramps 0 -> 1 in the background over the next few control-loop
     *    ticks (see DriverUnitreeG1ArmSDK::enable_arm_control()).
     *  - MODE::LOCOMOTION: calls arm_sdk_.deinitialize(), which synchronously ramps
     *    the blend weight 1 -> 0 -- holding the last commanded pose throughout -- and
     *    only returns once that ramp has actually completed, unlike
     *    disable_arm_control(), which only requests the ramp and returns immediately.
     *    Because deinitialize() also stops arm_sdk_'s background control thread,
     *    arm_sdk_.initialize() is called right after to restart it, so a later
     *    set_mode(MODE::ARM_CONTROL) still works. Only then is
     *    loco_client_.set_balance_mode(1) called: per the class-level @warning, the
     *    robot ignores velocity commands while the arm blend weight is above 0, so
     *    this restores that response once the weight has actually reached 0 -- not
     *    merely once disable_arm_control() was requested.
     *
     * @param mode The mode to switch to.
     * @throws std::logic_error if operation_mode_ != OPERATION_MODE::HIGH_LEVEL.
     * @throws std::invalid_argument if @p mode doesn't match either enumerator.
     */
    void set_mode(const HIGH_MODE& mode);

    // --- Getters -- delegate to loco_client_ (OPERATION_MODE::HIGH_LEVEL only) ---

    /// Returns the robot's current FSM (finite state machine) ID. @throws std::logic_error if not HIGH_LEVEL.
    int get_fsm_id();
    /// Returns the robot's current FSM mode. @throws std::logic_error if not HIGH_LEVEL.
    int get_fsm_mode();
    /// Returns the robot's current balance mode. @throws std::logic_error if not HIGH_LEVEL.
    int get_balance_mode();
    /// Returns the robot's current swing height, in meters. @throws std::logic_error if not HIGH_LEVEL.
    double get_swing_height();
    /// Returns the robot's current stand height, in meters. @throws std::logic_error if not HIGH_LEVEL.
    double get_stand_height();
    /// Returns the robot's current gait phase. @throws std::logic_error if not HIGH_LEVEL.
    std::vector<double> get_phase();

    // --- Setters -- delegate to loco_client_ (OPERATION_MODE::HIGH_LEVEL only) ---

    /// Sets the robot's FSM (finite state machine) ID. See DriverUnitreeLocoClient's warning on firmware-dependent FSM IDs. @throws std::logic_error if not HIGH_LEVEL.
    void set_fsm_id(const int& fsm_id);
    /// Sets the robot's balance mode. See DriverUnitreeLocoClient's warning on firmware-dependent balance-mode values. @throws std::logic_error if not HIGH_LEVEL.
    void set_balance_mode(const int& balance_mode);
    /// Sets the robot's swing height, in meters. @throws std::logic_error if not HIGH_LEVEL.
    void set_swing_height(const double& swing_height);
    /// Sets the robot's stand height, in meters. @throws std::logic_error if not HIGH_LEVEL.
    void set_stand_height(const double& stand_height);
    /// Sets the robot's speed mode. @throws std::logic_error if not HIGH_LEVEL.
    void set_speed_mode(int mode);
    /// Sets the robot's target high-level velocities (vx, vy, yaw_rate). Not a joint position; kept as std::array. @throws std::logic_error if not HIGH_LEVEL.
    void set_target_high_level_velocities(const std::array<double,3>& target_high_level_velocities);

    // --- Arm SDK control (delegates to arm_sdk_; OPERATION_MODE::HIGH_LEVEL only) ---

    /// Requests the arm-control blend weight to ramp up toward 1.0. See DriverUnitreeG1ArmSDK::enable_arm_control(). @throws std::logic_error if not HIGH_LEVEL.
    void enable_arm_control();
    /// Requests the arm-control blend weight to ramp down toward 0.0. See DriverUnitreeG1ArmSDK::disable_arm_control(). @throws std::logic_error if not HIGH_LEVEL.
    void disable_arm_control();
    /// Whether arm control is currently requested to be engaged. See DriverUnitreeG1ArmSDK::is_arm_control_enabled(). @throws std::logic_error if not HIGH_LEVEL.
    bool is_arm_control_enabled() const;

    /**
     * @brief Sets the left arm's 7 target joint positions, in radians.
     * @param target_positions Size-7 vector of target joint positions, in radians.
     * @throws std::invalid_argument if target_positions.size() != 7 (checked first,
     *         regardless of operation_mode_).
     * @throws std::logic_error if operation_mode_ == OPERATION_MODE::LOW_LEVEL --
     *         low-level joint commands require DriverUnitreeLowCmd, which isn't
     *         available yet.
     */
    void set_left_arm_target_positions(const Eigen::VectorXd& target_positions);
    /**
     * @brief Sets the right arm's 7 target joint positions, in radians.
     * @param target_positions Size-7 vector of target joint positions, in radians.
     * @throws std::invalid_argument if target_positions.size() != 7 (checked first,
     *         regardless of operation_mode_).
     * @throws std::logic_error if operation_mode_ == OPERATION_MODE::LOW_LEVEL --
     *         low-level joint commands require DriverUnitreeLowCmd, which isn't
     *         available yet.
     */
    void set_right_arm_target_positions(const Eigen::VectorXd& target_positions);
    /**
     * @brief Sets the waist's 3 target joint positions, in radians.
     * @param target_positions Size-3 vector of target joint positions, in radians.
     * @throws std::invalid_argument if target_positions.size() != 3 (checked first,
     *         regardless of operation_mode_).
     * @throws std::logic_error if operation_mode_ == OPERATION_MODE::LOW_LEVEL --
     *         low-level joint commands require DriverUnitreeLowCmd, which isn't
     *         available yet.
     */
    void set_waist_target_positions(const Eigen::VectorXd& target_positions);

    /// Returns the left arm's 7 measured joint positions, in radians. @throws std::logic_error if not HIGH_LEVEL.
    Eigen::VectorXd get_left_arm_positions();
    /// Returns the right arm's 7 measured joint positions, in radians. @throws std::logic_error if not HIGH_LEVEL.
    Eigen::VectorXd get_right_arm_positions();
    /// Returns the waist's 3 measured joint positions, in radians. @throws std::logic_error if not HIGH_LEVEL.
    Eigen::VectorXd get_waist_positions();

    /// Returns the left arm's 7 commanded trajectory-point positions, in radians. @throws std::logic_error if not HIGH_LEVEL.
    Eigen::VectorXd get_left_arm_desired_positions();
    /// Returns the right arm's 7 commanded trajectory-point positions, in radians. @throws std::logic_error if not HIGH_LEVEL.
    Eigen::VectorXd get_right_arm_desired_positions();
    /// Returns the waist's 3 commanded trajectory-point positions, in radians. @throws std::logic_error if not HIGH_LEVEL.
    Eigen::VectorXd get_waist_desired_positions();

    // --- Low-level state telemetry (delegates to low_state_; available in BOTH operation modes) ---
    // Read-only rt/lowstate data for every joint on the robot (not just the arms and
    // waist arm_sdk_ tracks) plus the onboard IMU. See DriverUnitreeLowState for the
    // per-joint indexing (matches the SDK's own motor_state() array order) and for
    // which joint count applies to which robot/message set. low_state_ is connected
    // and initialized regardless of operation_mode_, so none of the methods below
    // ever throw std::logic_error over the operation mode.

    /// Number of joint slots in the underlying rt/lowstate message (35 for G1). 0 if
    /// no message has been received yet.
    std::size_t get_num_joints();
    /// Returns every joint's last measured position, in radians. Empty if no
    /// rt/lowstate message has been received yet.
    Eigen::VectorXd get_all_joint_positions();
    /// Returns every joint's last measured velocity, in rad/s. Empty if no
    /// rt/lowstate message has been received yet.
    Eigen::VectorXd get_all_joint_velocities();
    /// Returns every joint's last estimated torque, in Nm. Empty if no rt/lowstate
    /// message has been received yet.
    Eigen::VectorXd get_all_joint_torques();
    /// Returns the last received IMU reading. Not a joint position; kept as
    /// DriverUnitreeLowState::IMUData (see that class for its fields).
    DriverUnitreeLowState::IMUData get_imu_data();

    // --- Per-limb (and torso) telemetry (delegates to low_state_; available in BOTH operation modes) ---
    // Reuses DriverUnitreeLowState::LIMB directly rather than duplicating it as a
    // separate DriverUnitreeG1::LIMB: they'd otherwise be two enums needing to stay
    // in sync, for no benefit -- the same choice already made for
    // DriverUnitreeLowState::IMUData above. Joint count and indexing per (robot,
    // limb) are documented on DriverUnitreeLowState's own class-level @note; these
    // three getters already return Eigen::VectorXd on DriverUnitreeLowState itself
    // (unlike its full-body get_joint_positions()/etc., which return
    // std::vector<double>), so no conversion is needed here -- they forward
    // directly.

    /// Number of joints in the given limb (or torso) (e.g. 7 for the left arm on
    /// G1). Does not require an rt/lowstate message to have been received yet.
    std::size_t num_joints(const DriverUnitreeLowState::LIMB& limb) const;
    /// Returns the given limb's (or the torso's) last measured joint positions, in
    /// radians. Empty if no rt/lowstate message has been received yet.
    Eigen::VectorXd get_joint_positions(const DriverUnitreeLowState::LIMB& limb) const;
    /// Returns the given limb's (or the torso's) last measured joint velocities, in
    /// rad/s. Empty if no rt/lowstate message has been received yet.
    Eigen::VectorXd get_joint_velocities(const DriverUnitreeLowState::LIMB& limb) const;
    /// Returns the given limb's (or the torso's) last estimated joint torques, in
    /// Nm. Empty if no rt/lowstate message has been received yet.
    Eigen::VectorXd get_joint_torques(const DriverUnitreeLowState::LIMB& limb) const;
};
