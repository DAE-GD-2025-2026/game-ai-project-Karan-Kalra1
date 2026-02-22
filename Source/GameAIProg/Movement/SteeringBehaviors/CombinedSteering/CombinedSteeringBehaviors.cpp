
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

// =======================
// BLENDED STEERING
// =======================

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
    : WeightedBehaviors(WeightedBehaviors)
{
}

SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput result{};

    FVector2D totalLinear = FVector2D::ZeroVector;
    float totalAngular = 0.f;

    for (const WeightedBehavior& wb : WeightedBehaviors)
    {
        if (!wb.pBehavior) continue;

        SteeringOutput steering = wb.pBehavior->CalculateSteering(DeltaT, Agent);

        if (!steering.IsValid) continue;

        totalLinear += steering.LinearVelocity * wb.Weight;
        totalAngular += steering.AngularVelocity * wb.Weight;
    }

    result.LinearVelocity = totalLinear;
    result.AngularVelocity = totalAngular;
    result.IsValid = true;

    return result;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
    auto it = std::find_if(
        WeightedBehaviors.begin(),
        WeightedBehaviors.end(),
        [SteeringBehavior](const WeightedBehavior& Elem)
        {
            return Elem.pBehavior == SteeringBehavior;
        });

    if (it != WeightedBehaviors.end())
        return &it->Weight;

    return nullptr;
}

// =======================
// PRIORITY STEERING
// =======================

SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};

    for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
    {
        Steering = pBehavior->CalculateSteering(DeltaT, Agent);


        if (Steering.IsValid)
            return Steering;
    }

    return Steering;
}