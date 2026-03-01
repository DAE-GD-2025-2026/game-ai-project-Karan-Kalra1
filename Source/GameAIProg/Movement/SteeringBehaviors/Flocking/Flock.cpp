#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);

	// Initialize memory pool
	Neighbors.SetNum(FlockSize);



	//Initialize PartitionedSpace
	pPartitionedSpace = std::make_unique<CellSpace>(
		pWorld,
		WorldSize,
		WorldSize,
		10,
		10,
		FlockSize
	);

	// Spawn agents
	for (int i = 0; i < FlockSize; ++i)
	{
		FVector spawnPos = FVector(
			FMath::RandRange(-WorldSize, WorldSize),
			FMath::RandRange(-WorldSize, WorldSize),
			90.f);

		ASteeringAgent* agent =
			pWorld->SpawnActor<ASteeringAgent>(AgentClass, spawnPos, FRotator::ZeroRotator);

		if (!agent)
		{
			i--;
			continue;
		}

		agent->SetActorTickEnabled(false);

		pPartitionedSpace->AddAgent(*agent);

		Agents[i] = agent;

		
	}

	pCohesion = std::make_unique<Cohesion>(this);
	pSeparation = std::make_unique<Separation>(this);
	pVelocityMatch = std::make_unique<VelocityMatch>(this);
	pWander = std::make_unique<Wander>();
	pSeek = std::make_unique<Seek>();

	std::vector<BlendedSteering::WeightedBehavior> behaviors =
	{
		{pSeek.get(), 0.1f},
		{ pCohesion.get(), 0.3f },
		{ pSeparation.get(), 0.34f },
		{ pVelocityMatch.get(), 0.5f },
		{ pWander.get(), 0.2f }
	};

	pBlendedSteering = std::make_unique<BlendedSteering>(behaviors);

	if (pAgentToEvade)
	{
		pEvade = std::make_unique<Evade>(pAgentToEvade);

		std::vector<ISteeringBehavior*> priorityBehaviors =
		{
			pEvade.get(),
			pBlendedSteering.get()
		};

		pPrioritySteering = std::make_unique<PrioritySteering>(priorityBehaviors);
	}

	for (ASteeringAgent* agent : Agents)
	{
		

		if (pPrioritySteering)
			agent->SetSteeringBehavior(pPrioritySteering.get());
		else
			agent->SetSteeringBehavior(pBlendedSteering.get());
	}


}

Flock::~Flock()
{
 // TODO: Cleanup any additional data

}

void Flock::Tick(float DeltaTime)
{
	if (bUseSpatialPartitioning)
	{
		pPartitionedSpace->EmptyCells();

		for (ASteeringAgent* agent : Agents)
			pPartitionedSpace->AddAgent(*agent);
	}


		for (ASteeringAgent* agent : Agents)
		{
			if (bUseSpatialPartitioning)
			{
				pPartitionedSpace->RegisterNeighbors(*agent, NeighborhoodRadius);

				NrOfNeighbors = pPartitionedSpace->GetNrOfNeighbors();
				const TArray<ASteeringAgent*>& partitionNeighbors =
					pPartitionedSpace->GetNeighbors();

				for (int i = 0; i < NrOfNeighbors; ++i)
				{
					Neighbors[i] = partitionNeighbors[i];
				}
			}
			else
			{
				RegisterNeighbors_NoPartition(agent);
			}

			agent->Tick(DeltaTime);
		}

		if (!bUseSpatialPartitioning)
			DebugRenderPartitions = false;

		//UE_LOG(LogTemp, Warning, TEXT("Neighbors: %d"), NrOfNeighbors);
}

void Flock::RenderDebug()
{
 for (ASteeringAgent* agent : Agents)
	{
	 agent->SetDebugRenderingEnabled(DebugRenderSteering);
	}

if (DebugRenderNeighborhood)
    RenderNeighborhood();

if (DebugRenderPartitions && bUseSpatialPartitioning)
pPartitionedSpace->RenderCells();

}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

  // TODO: implement ImGUI checkboxes for debug rendering here

		 ImGui::Checkbox("Debug Steering", &DebugRenderSteering);
		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);
		ImGui::Checkbox("Debug Partitions", &DebugRenderPartitions);

		ImGui::Separator();
		ImGui::Checkbox("Use Spatial Partitioning", &bUseSpatialPartitioning);

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
		

		if (pBlendedSteering)
		{
			auto& behaviors = pBlendedSteering->GetWeightedBehaviorsRef();

			for (int i = 0; i < behaviors.size(); ++i)
			{
				float weight = behaviors[i].Weight;

				FString label;

				switch (i)
				{
				case 0: label = "Seek"; break;
				case 1: label = "Cohesion"; break;
				case 2: label = "Separation"; break;
				case 3: label = "Alignment"; break;
				case 4: label = "Wander"; break;
				default: label = "Behavior"; break;
				}

				if (ImGui::SliderFloat(
					TCHAR_TO_ANSI(*label),
					&weight,
					0.f,
					2.f,
					"%.2f"))
				{
					behaviors[i].Weight = weight;
				}
			}
		}
  
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	if (Agents.Num() == 0)
		return;

	ASteeringAgent* first = Agents[0];

	DrawDebugCircle(
		pWorld,
		FVector(first->GetPosition(), 0.f),
		NeighborhoodRadius,
		32,
		FColor::Green,
		false,
		-1.f,
		0,
		2.f,
		FVector(1, 0, 0),
		FVector(0, 1, 0),
		false);
}


void Flock::RegisterNeighbors_NoPartition(ASteeringAgent* const pAgent)
{
	NrOfNeighbors = 0;

	for (ASteeringAgent* other : Agents)
	{
		

		if (other == pAgent)
			continue;

		float distance = FVector2D::Distance(
			pAgent->GetPosition(),
			other->GetPosition());

		if (distance < NeighborhoodRadius)
		{
			Neighbors[NrOfNeighbors] = other;
			NrOfNeighbors++;
		}
	}
}


FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

	if (NrOfNeighbors == 0)
		return FVector2D::ZeroVector;

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		avgPosition += Neighbors[i]->GetPosition();
	}

	return avgPosition / NrOfNeighbors;
	
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	if (NrOfNeighbors == 0)
		return FVector2D::ZeroVector;

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		avgVelocity += Neighbors[i]->GetLinearVelocity();
	}

	return avgVelocity / NrOfNeighbors;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	
		if (pSeek)
		{
			pSeek->SetTarget(Target);
		}
	
}

