#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};


	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	

	//Add debug Rendering for grades

	return Steering;
}


SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{

	SteeringOutput Steering{};


	Steering.LinearVelocity = -(Target.Position - Agent.GetPosition());

	//Add debug Rendering for grades

	return Steering;


}


Arrive::Arrive(ASteeringAgent& Agent)
{
	MaxSpeed = Agent.GetMaxLinearSpeed();
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{

	SteeringOutput Steering{};

	FVector2D toTarget = Target.Position - Agent.GetPosition();

	double distance = toTarget.Length();

	if (distance < TargetRadius)
	{
		Steering.LinearVelocity = FVector2D::ZeroVector;
		return Steering;
	}

	double targetSpeed = 0.0;


	if (distance > SlowRadius)
	{
		targetSpeed = MaxSpeed;  
	}

	else
	{
		
		targetSpeed = MaxSpeed * (distance / SlowRadius);
	}

	
	FVector2D desiredVelocity = toTarget.GetSafeNormal() * targetSpeed;

	Steering.LinearVelocity = desiredVelocity - Agent.GetLinearVelocity();

	return Steering;

}