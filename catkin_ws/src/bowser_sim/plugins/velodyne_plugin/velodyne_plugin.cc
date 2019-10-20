#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
            // Safety check
        if (_model->GetJointCount() == 0)
        {
          std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
          return;
        }

        // Store the model pointer for convenience.
        this->model = _model;

        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        this->joint = _model->GetJoints()[0];

        // Setup a P-controller, with a gain of 0.1.
        this->pid = common::PID(0.1, 0, 0);

        // Apply the P-controller to the joint.
        this->model->GetJointController()->SetVelocityPID(
            this->joint->GetScopedName(), this->pid);

        double velocity = 0;

        if (_sdf->HasElement("velocity"))
            velocity = _sdf->Get<double>("velocity");

        // Set the joint's target velocity. This target velocity is just
        // for demonstration purposes.
        this->model->GetJointController()->SetVelocityTarget(
            this->joint->GetScopedName(), velocity);

    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;



  };

  

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
