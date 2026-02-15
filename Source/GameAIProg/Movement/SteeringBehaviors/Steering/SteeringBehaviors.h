#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }
	
	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	FTargetData Target;
};

// Your own SteeringBehaviors should follow here...
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() override = default;


	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};


class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() override = default;


	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};


class Arrive : public ISteeringBehavior
{
public:
	Arrive(ASteeringAgent& Agent);
	virtual ~Arrive() override = default;


	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

private : 
	float SlowRadius{500.f};
	float TargetRadius{100.f};
	float MaxSpeed{};

};


class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() override = default;


	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

private:
	float RotationSpeed{ 5.f };
};

class Pursuit : public ISteeringBehavior
{
public:
	Pursuit() = default;
	virtual ~Pursuit() override = default;


	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

private:
	float RotationSpeed{ 5.f };
};

class Evade : public ISteeringBehavior
{
public:
	Evade() = default;
	virtual ~Evade() override = default;


	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

private:
	float RotationSpeed{ 1.f };
};

class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() override = default;


	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

private:

	float Offset{ 150.f };
	float Radius { 80.f };
	float MaxAngleChange{ 0.4f };
	float WanderAngle = { 0.f };
};

