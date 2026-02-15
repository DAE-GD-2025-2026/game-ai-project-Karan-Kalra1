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


SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	FVector2D direction = Target.Position - Agent.GetPosition();

	if (direction.IsNearlyZero())
		return Steering;

	float targetAngle = FMath::Atan2(direction.Y, direction.X);
	float currentAngle = Agent.GetRotation();

	float angleDifference =
		FMath::FindDeltaAngleRadians(currentAngle, targetAngle);

	Steering.AngularVelocity = angleDifference * RotationSpeed;

	Steering.LinearVelocity = FVector2D::ZeroVector;

	return Steering;
}


SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	FVector2D toTarget = Target.Position - Agent.GetPosition();

	float distance = toTarget.Length();
	float speed = Agent.GetMaxLinearSpeed();

	float predictionTime = 0.f;

	if (speed > 0.1f)
		predictionTime = distance / speed;

	FVector2D predictedPosition =
		Target.Position + Target.LinearVelocity * predictionTime;

	FVector2D desired =
		(predictedPosition - Agent.GetPosition()).GetSafeNormal();

	Steering.LinearVelocity = desired;

	return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	FVector2D toTarget = Target.Position - Agent.GetPosition();

	float distance = toTarget.Length();
	float speed = Agent.GetMaxLinearSpeed();

	float predictionTime = (speed > 0.1f)
		? distance / speed
		: 0.f;

	FVector2D predictedPosition =
		Target.Position + Target.LinearVelocity * predictionTime;

	FVector2D desired =
		(Agent.GetPosition() - predictedPosition).GetSafeNormal();

	Steering.LinearVelocity = desired;

	return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// random angle offset
	WanderAngle += FMath::RandRange(-MaxAngleChange, MaxAngleChange);

	FVector2D forward = FVector2D(Agent.GetActorForwardVector());

	FVector2D circleCenter =
		Agent.GetPosition() + forward * Offset;

	FVector2D displacement(
		FMath::Cos(WanderAngle),
		FMath::Sin(WanderAngle));

	displacement *= Radius;

	Target.Position = circleCenter + displacement;

	DrawDebugCircle(
		Agent.GetWorld(),
		FVector(circleCenter, 0),
		Radius,
		32,
		FColor::Green,
		false,
		-1.f,
		0,
		2.f,
		FVector(1, 0, 0),   // X axis
		FVector(0, 1, 0),   // Y axis
		false
	);

	DrawDebugPoint(Agent.GetWorld(),
		FVector(Target.Position,0),
		4,
		FColor::Blue,
		false	
		);



	// reuse SEEK
	return Seek::CalculateSteering(DeltaT, Agent);
}


