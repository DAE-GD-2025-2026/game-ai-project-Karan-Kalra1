#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	Agent.SetIsAutoOrienting(true);

	SteeringOutput Steering{};



	FVector2D direction = Target.Position - Agent.GetPosition();

	float length = direction.SquaredLength();

	direction.Normalize();
		
	Steering.LinearVelocity = direction;

	

	//Add debug Rendering for grades

	if (Agent.GetDebugRenderingEnabled() && length > 100)
	{

		DrawDebugLine(Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0),
			FVector(Agent.GetPosition() + direction * 200, 0),
			FColor::Green,
			false,
			-1.f,
			0,
			3.f

		);
	}


	return Steering;
}


SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(true);

	SteeringOutput Steering{};


	FVector2D direction = -(Target.Position - Agent.GetPosition());

	float length = direction.SquaredLength();

	direction.Normalize();

	Steering.LinearVelocity = direction;

	

	//Add debug Rendering for grades
	if (Agent.GetDebugRenderingEnabled() && length > 100)
	{

		DrawDebugLine(Agent.GetWorld(),
			FVector(Agent.GetPosition() - direction * 200, 0),
			FVector(Agent.GetPosition(), 0),
			FColor::Green,
			false,
			-1.f,
			0,
			3.f

		);
	}

	return Steering;


}


Arrive::Arrive(ASteeringAgent& Agent)
{
	MaxSpeed = Agent.GetMaxLinearSpeed();
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(true);

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

	Steering.LinearVelocity.Normalize();

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0),
			SlowRadius,
			32,
			FColor::Red,
			false,
			-1.f,
			0,
			2.f,
			FVector(1, 0, 0),   // X axis
			FVector(0, 1, 0),   // Y axis
			false
		);

		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0),
			TargetRadius,
			32,
			FColor::Blue,
			false,
			-1.f,
			0,
			2.f,
			FVector(1, 0, 0),   // X axis
			FVector(0, 1, 0),   // Y axis
			false
		);


		DrawDebugLine(Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0),
			FVector(Target.Position, 0),
			FColor::Green,
			false,
			-1.f,
			0,
			2.f

		);


	}



	return Steering;

}


SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(false);

	SteeringOutput Steering{};

	FVector2D direction = Target.Position - Agent.GetPosition();


	float targetAngle = FMath::Atan2(direction.Y, direction.X);

	float currentAngle = FMath::DegreesToRadians(Agent.GetRotation());

	float angleDifference =
		FMath::FindDeltaAngleRadians(currentAngle, targetAngle);

	Steering.AngularVelocity = angleDifference * RotationSpeed;

	Steering.LinearVelocity = FVector2D::ZeroVector;


	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(Target.Position, 0),
			10,
			32,
			FColor::Red,
			false,
			-1.f,
			0,
			2.f,
			FVector(1, 0, 0),   // X axis
			FVector(0, 1, 0),   // Y axis
			false
		);

	}

	return Steering;
}


SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(true);

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


	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugLine(Agent.GetWorld(),
			FVector(Agent.GetPosition(),0),
			FVector(Agent.GetPosition() + toTarget.GetSafeNormal() * 200, 0),
			FColor::Green,
			false,
			-1.f,
			0,
			3.f

		);


		DrawDebugLine(Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0),
			FVector(Agent.GetPosition() + desired * 200, 0),
			FColor::Blue,
			false,
			-1.f,
			0,
			3.f

		);

	}

	return Steering;
}

Evade::Evade(ASteeringAgent* Agent)
{

	Target.Position = Agent->GetPosition();
	Target.Orientation = Agent->GetRotation();
	Target.LinearVelocity = Agent->GetLinearVelocity();
	Target.AngularVelocity = Agent->GetAngularVelocity();

}

void Evade::SetEvadeDistance(float distance)
{
	_EvadeDistance = distance;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(true);

	SteeringOutput Steering{};

	Steering.IsValid = true;

		FVector2D toTarget = Target.Position - Agent.GetPosition();

		float distance = toTarget.Length();
		float speed = Agent.GetMaxLinearSpeed();

		if (distance > _EvadeDistance && _EvadeDistance > 0)
		{
			Steering.IsValid = false;
		}

		float predictionTime = (speed > 0.1f)
			? distance / speed
			: 0.f;

		FVector2D predictedPosition =
			Target.Position + Target.LinearVelocity * predictionTime;

		FVector2D desired =
			(Agent.GetPosition() - predictedPosition).GetSafeNormal();

		Steering.LinearVelocity = desired;
	

	if (Agent.GetDebugRenderingEnabled() && Steering.IsValid)
	{
		DrawDebugLine(Agent.GetWorld(),
			FVector(Agent.GetPosition() - toTarget.GetSafeNormal() * 200, 0),
			FVector(Agent.GetPosition(), 0),
			FColor::Green,
			false,
			-1.f,
			0,
			3.f

		);


		DrawDebugLine(Agent.GetWorld(),
			FVector(Agent.GetPosition() + desired * 200, 0),
			FVector(Agent.GetPosition(), 0),
			FColor::Blue,
			false,
			-1.f,
			0,
			3.f

		);

		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0),
			_EvadeDistance,
			32,
			FColor::Red,
			false,
			-1.f,
			0,
			2.f,
			FVector(1, 0, 0),   // X axis
			FVector(0, 1, 0),   // Y axis
			false
		);

	}



	return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetIsAutoOrienting(true);

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

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(
			Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0),
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
			FVector(Agent.GetPosition() + displacement, 0),
			4,
			FColor::Blue,
			false
		);

	}

	// reuse SEEK
	return Seek::CalculateSteering(DeltaT, Agent);
}


