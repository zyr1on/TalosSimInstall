#ifndef CART_PLUGIN_HH_
#define CART_PLUGIN_HH_

#include <ignition/math/Vector3.hh>
#include <ignition/msgs/cmd_vel2d.pb.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <cart_sim/cart_control.h>


namespace gazebo
{
  // Forward declaration
  class CartPluginPrivate;

  /// \brief A model plugin for cart vehicle
  class CartPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: CartPlugin();

    /// \brief Destructor.
    public: virtual ~CartPlugin();

    // Documentation Inherited
    public: virtual void Reset();

    /// \brief Load the controller.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback each time a key message is received.
    /// \param[in] _msg Keypress message.
    private: void OnKeyPress(ConstAnyPtr &_msg);
    private: void OnCartCommand(const cart_sim::cart_control::ConstPtr &msg);

    /// \brief Callback each time a key message is received.
    /// \param[in] _msg Keypress message.
    private: void OnKeyPressIgn(const ignition::msgs::Any &_msg);

    /// \brief Key control
    /// \param[in] _key key value
    private: void KeyControl(const int _key);

    /// \brief Key control type A
    /// \param[in] _key key value
    private: void KeyControlTypeA(const int _key);

    /// \brief Key control type B
    /// \param[in] _key key value
    private: void KeyControlTypeB(const int _key);

    /// \param[in] _msg Pose message
    private: void OnCmdVel(const ignition::msgs::Pose &_msg);

    /// \brief Command to change gear to reverse, neutral or forward (drive)
    /// \param[in] _msg Int32 message data
    private: void OnCmdGear(const ignition::msgs::Int32 &_msg);

    /// \brief Command to enable EV mode
    /// \param[in] _msg Boolean message data
    private: void OnCmdMode(const ignition::msgs::Boolean &_msg);

    /// \brief Command to reset the world
    /// \param[in] _msg Int32 message data. Not used
    private: void OnReset(const ignition::msgs::Any &_msg);

    /// \brief Command to stop the simulation
    /// \param[in] _msg Int32 message data. Not used
    private: void OnStop(const ignition::msgs::Any &_msg);

    /// \brief Update on every time step
    private: void Update();

    /// \brief Update steering wheel to front left/right wheel ratio
    private: void UpdateHandWheelRatio();

    /// \brief Get the radius of a collision
    private: double CollisionRadius(physics::CollisionPtr _collision);

    /// \brief Get the multiplier that is determined based on the direction
    /// state of the vehicle.
    /// \return 1.0 if FORWARD, -1.0 if REVERSE, 0.0 otherwise
    private: double GasTorqueMultiplier();

    /// \brief Private data
    private: std::unique_ptr<CartPluginPrivate> dataPtr;

    /// ROS Namespace
    private: std::string robot_namespace_;
  };
}
#endif
