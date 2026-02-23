#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput result{};

    FVector2D avgPos = pFlock->GetAverageNeighborPos();


    SetTarget(FSteeringParams(avgPos));

    return Seek::CalculateSteering(deltaT, pAgent);
}
//*********************
//SEPARATION (FLOCKING)

SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput result{};
    FVector2D steering{};

    const TArray<ASteeringAgent*>& neighbors = pFlock->GetNeighbors();
    int count = pFlock->GetNrOfNeighbors();

    for (int i = 0; i < count; ++i)
    {
        FVector2D toAgent =
            pAgent.GetPosition() - neighbors[i]->GetPosition();

        float distSq = toAgent.SquaredLength ();

            steering += toAgent / distSq;
    }

    result.LinearVelocity = steering;

    result.LinearVelocity.Normalize();

    return result;
}

//*************************
//VELOCITY MATCH (FLOCKING)

SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput result{};

    FVector2D avgVelocity = pFlock->GetAverageNeighborVelocity();

    result.LinearVelocity = avgVelocity.GetSafeNormal();

    return result;
}