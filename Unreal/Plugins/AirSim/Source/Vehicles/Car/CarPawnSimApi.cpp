#include "CarPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "CarPawnApi.h"
#include <exception>

using namespace msr::airlib;

CarPawnSimApi::CarPawnSimApi(ACarPawn* pawn, const NedTransform& global_transform, PawnEvents* pawn_events,
    const common_utils::UniqueValueMap<std::string, APIPCamera*>& cameras, UClass* pip_camera_class, UParticleSystem* collision_display_template,
    const CarPawnApi::CarControls&  keyboard_controls,
    UWheeledVehicleMovementComponent* movement, const msr::airlib::GeoPoint& home_geopoint)
    : PawnSimApi(pawn, global_transform, pawn_events, cameras, pip_camera_class, collision_display_template, home_geopoint),
      keyboard_controls_(keyboard_controls)
{
    createVehicleApi(pawn, home_geopoint);

    //TODO: should do reset() here?
    joystick_controls_ = CarPawnApi::CarControls();
}

void CarPawnSimApi::createVehicleApi(ACarPawn* pawn, const msr::airlib::GeoPoint& home_geopoint)
{
    //create vehicle params
    //std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_api_ = std::unique_ptr<CarApiBase>(new CarPawnApi(pawn, getPawnKinematics(), home_geopoint));
}

std::string CarPawnSimApi::getRecordFileLine(bool is_header_line) const
{
    std::string common_line = PawnSimApi::getRecordFileLine(is_header_line);
    if (is_header_line) {
        return common_line +
               "Throttle\tSteering\tBrake\tGear\tHandbrake\tRPM\tSpeed\t";
    }

    const msr::airlib::Kinematics::State* kinematics = getGroundTruthKinematics();
    const auto state = vehicle_api_->getCarState();

    common_line
        .append(std::to_string(current_controls_.throttle)).append("\t")
        .append(std::to_string(current_controls_.steering)).append("\t")
        .append(std::to_string(current_controls_.brake)).append("\t")
        .append(std::to_string(state.gear)).append("\t")
        .append(std::to_string(state.handbrake)).append("\t")
        .append(std::to_string(state.rpm)).append("\t")
        .append(std::to_string(state.speed)).append("\t")
        ;

    return common_line;
}

//these are called on render ticks
void CarPawnSimApi::updateRenderedState(float dt)
{
    PawnSimApi::updateRenderedState(dt);
    
    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    //TODO: do we need this for cars?
    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
}
void CarPawnSimApi::updateRendering(float dt)
{
    PawnSimApi::updateRendering(dt);

    updateCarControls();

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }
}

#define AUTOCENTERGAIN 0.55
#define DAMPERGAIN 0.4

void CarPawnSimApi::updateCarControls()
{
    auto rc_data = getRCData();

    if (rc_data.is_initialized) {
        if (!rc_data.is_valid) {
            UAirBlueprintLib::LogMessageString("Control Mode: ", "[INVALID] Wheel/Joystick", LogDebugLevel::Informational);
            return;
        }
        UAirBlueprintLib::LogMessageString("Control Mode: ", "Wheel/Joystick", LogDebugLevel::Informational);

        //TODO: move this to SimModeBase?
        //if ((joystick_state_.buttons & 4) | (joystick_state_.buttons & 1024)) { //X button or Start button
        //    reset();
        //    return;
        //}

        // Thrustmaster devices
        if (rc_data.vendor_id == "VID_044F") {
            joystick_controls_.steering = rc_data.yaw * 1.25;
            joystick_controls_.throttle = (-rc_data.right_z + 1) / 2;
            joystick_controls_.brake = rc_data.throttle;

			auto carpawn = dynamic_cast<ACarPawn*>(getPawn());
			if (carpawn != nullptr) {
				std::unique_ptr<ACarPawn::HitUtilities>& hitUtilities_ = carpawn->GetHitUtilities();

				// Reminder: (from SimJoyStick.h)
				// AutoCenter strength ranges from -1 to 1
				// WheelRumble strength ranges from 0 to 1
				// Damper strength ranges from 0 to 1
				// HitEffect strength ranges from -1 to 1

				float rumble_strength = 0.5 - (carpawn->GetVehicleMovement()->GetEngineRotationSpeed()
					/ carpawn->GetVehicleMovement()->GetEngineMaxRotationSpeed()) / 3;

				double speedcms = carpawn->GetVehicleMovement()->GetForwardSpeed();// cm/s
				double speedkmh = speedcms * 0.036;// km/h

				float damper_strength = DAMPERGAIN * std::min(1.0, (std::abs(speedkmh) / 60.0)); 

				// Hit or autocenter, not both
				if (hitUtilities_ == nullptr || !hitUtilities_->IsHitPhysicalEffectOn()) {
					float steeringSign = joystick_controls_.steering >= 0 ? 1.0 : -1.2;//compensate for biased steering
					float autocenter_strength = std::min(1.0,(0.3 + std::abs(speedkmh) / 60.0)) * std::sqrt(std::abs(joystick_controls_.steering * 2.0)) * steeringSign * AUTOCENTERGAIN;

					UAirBlueprintLib::LogMessageString("Hit:", "hit off", LogDebugLevel::Informational);
					
					// 0 if no collide 
					if (hitUtilities_ != nullptr && !hitUtilities_->IsHitvirtualEffectOn()) {
						// must be released, not active
						hitUtilities_.reset(nullptr);
					}

					setRCForceFeedback(rumble_strength, autocenter_strength, damper_strength, 0, true);
				}
				else {
					UAirBlueprintLib::LogMessageString("Hit:", "hit on", LogDebugLevel::Informational);

					if (hitUtilities_->freshHit) {
						hitUtilities_->freshHit = false;
						hitUtilities_->hitSpeed = speedcms;
					}

					float hit_strength = hitUtilities_->GetDirectionSign() * hitUtilities_->hitStrength
						* ((std::abs(hitUtilities_->hitSpeed) + 80) / 60);

					setRCForceFeedback(rumble_strength, 0, damper_strength, hit_strength, false);
				}
			}
        }
        // Anything else, typically Logitech G920 wheel
        else {
            joystick_controls_.steering = (rc_data.throttle * 2 - 1) * 1.25;
            joystick_controls_.throttle = (-rc_data.roll + 1) / 2;
            joystick_controls_.brake = -rc_data.right_z + 1;
        }
        //Two steel levers behind wheel
        //joystick_controls_.handbrake = (rc_data.getSwitch(5)) | (rc_data.getSwitch(6)) ? 1 : 0;

		auto timeNow = scalableClock.nowNanos();
		if (msr::airlib::ClockBase::elapsedBetween(timeNow, prevTime) * 1.0E3 > 256)
		{
			if ((rc_data.getSwitch(1)) | (rc_data.getSwitch(0))) { //RSB button or B button
				joystick_controls_.manual_gear = current_controls_.manual_gear < 0 ? 0 : -1;
				joystick_controls_.is_manual_gear = !current_controls_.manual_gear;
				joystick_controls_.gear_immediate = true;
			}
			prevTime = timeNow;
		}

        current_controls_ = joystick_controls_;
    }
    else {
        UAirBlueprintLib::LogMessageString("Control Mode: ", "Keyboard", LogDebugLevel::Informational);
        current_controls_ = keyboard_controls_;
    }

    //if API-client control is not active then we route keyboard/joystick control to car
    if (!vehicle_api_->isApiControlEnabled()) {
        //all car controls from anywhere must be routed through API component
        vehicle_api_->setCarControls(current_controls_);
    }
    else {
        UAirBlueprintLib::LogMessageString("Control Mode: ", "API", LogDebugLevel::Informational);
        current_controls_ = vehicle_api_->getCarControls();
    }
    UAirBlueprintLib::LogMessageString("Accel: ", std::to_string(current_controls_.throttle), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Break: ", std::to_string(current_controls_.brake), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Steering: ", std::to_string(current_controls_.steering), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Handbrake: ", std::to_string(current_controls_.handbrake), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Target Gear: ", std::to_string(current_controls_.manual_gear), LogDebugLevel::Informational);
}

//*** Start: UpdatableState implementation ***//
void CarPawnSimApi::reset()
{
    PawnSimApi::reset();

    vehicle_api_->reset();
}

//physics tick
void CarPawnSimApi::update()
{
    vehicle_api_->update();

    PawnSimApi::update();
}

void CarPawnSimApi::reportState(StateReporter& reporter)
{
    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    FVector unrealPosition = getUUPosition();
    reporter.writeValue("unreal pos", Vector3r(unrealPosition.X, unrealPosition.Y, unrealPosition.Z));
}
//*** End: UpdatableState implementation ***//

