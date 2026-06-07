// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_GraphTheory.h"

#include "Algorithms/EulerianPath.h"
#include "Shared/GameAISpectator.h"
#include <set>

using namespace GameAI;

// Sets default values
ALevel_GraphTheory::ALevel_GraphTheory()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_GraphTheory::BeginPlay()
{
	Super::BeginPlay();

	Renderer = GraphRenderer(GetWorld());
	
	// Add the graph editor to our player
	if (PlayerController = Cast<APlayerController>(GetWorld()->GetFirstLocalPlayerFromController()->PlayerController); 
		GraphEditorClass && PlayerController)
	{
		PlayerGraphEditor = NewObject<UGraphEditorComponent>(PlayerController->GetPawn(), GraphEditorClass);
		PlayerGraphEditor->RegisterComponent();
		PlayerGraphEditor->SetEditedGraph(&Graph);
		PlayerGraphEditor->SetNodeFactory(&NodeFactory);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Unable to get PlayerController from LocalPlayer or GraphEditorClass is null"))
		return;
	}
	
	// Make the view orthogonal for less perspective issues
	if (AGameAISpectator* Player = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
	{
		Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
	}
	
	// TODO Make the graph and a couple connected nodes here...
	// TODO Make the graph and a couple connected nodes here...
	int n0 = Graph.AddNode(std::make_unique<Node>(FVector2D{ 0.f, 0.f }));
	int n1 = Graph.AddNode(std::make_unique<Node>(FVector2D{ 250.f, 0.f }));
	int n2 = Graph.AddNode(std::make_unique<Node>(FVector2D{ 125.f, 220.f }));
	int n3 = Graph.AddNode(std::make_unique<Node>(FVector2D{ 0.f, 220.f }));
	int n4 = Graph.AddNode(std::make_unique<Node>(FVector2D{ 250.f, 220.f }));

	Graph.AddConnection(n0, n1);
	Graph.AddConnection(n0, n3);
	Graph.AddConnection(n0, n4);
	Graph.AddConnection(n1, n3);
	Graph.AddConnection(n1, n4);
	Graph.AddConnection(n2, n3);
	Graph.AddConnection(n2, n4);
	Graph.AddConnection(n3, n4);

	// Spawn the Agent
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
	FVector{0,0,90}, FRotator::ZeroRotator);
	Agent->SetSteeringBehavior(&PathFollow);
}

void ALevel_GraphTheory::BeginDestroy()
{
	Super::BeginDestroy();
}

void ColorGraph(GameAI::Graph& graph)
{
	auto nodes = graph.GetActiveNodes();

	//optionally sort by degree
	std::sort(nodes.begin(), nodes.end(),
		[&](GameAI::Node* a, GameAI::Node* b)
		{
			return graph.FindConnectionsFrom(a->GetId()).size() >
				graph.FindConnectionsFrom(b->GetId()).size();
		});

	for (GameAI::Node* node : nodes)
	{
		std::set<int> usedColors;

		auto connections = graph.FindConnectionsFrom(node->GetId());

		for (auto* conn : connections)
		{
			int neighborId = conn->GetToId();
			auto& neighborPtr = graph.GetNode(neighborId);

			if (neighborPtr && neighborPtr->Color != -1)
			{
				usedColors.insert(neighborPtr->Color);
			}
		}

		int color = 0;
		while (usedColors.contains(color))
		{
			color++;
		}

		node->Color = color;
	}
}


FColor GetDebugColor(int color)
{
	static std::vector<FColor> palette =
	{
		FColor::Red,
		FColor::Green,
		FColor::Blue,
		FColor::Yellow,
		FColor::Cyan,
		FColor::Magenta
	};

	return palette[color % palette.size()];
}

void ALevel_GraphTheory::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

#pragma region UI
	{
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::SetWindowFocus();
		ImGui::PushItemWidth(70);

		ImGui::Text("CONTROLS");
		ImGui::Indent();
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
		ImGui::Spacing();

		ImGui::Text("Graph Theory");
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::End();
	}
#pragma endregion UI

	Renderer.RenderGraph(Graph);
	ColorGraph(Graph);

	for (GameAI::Node* node : Graph.GetActiveNodes())
	{
		FColor color = GetDebugColor(node->Color);

		DrawDebugSphere(
			GetWorld(),
			FVector(node->GetPosition(), 0.f),
			32.f,
			8,
			color,
			false,
			-1.f
		);
	}

	int maxColor = 0;
	for (auto* node : Graph.GetActiveNodes())
	{
		maxColor = FMath::Max(maxColor, node->Color);
	}

	UE_LOG(LogTemp, Warning, TEXT("Colors used: %d"), maxColor + 1);


	static int PreviousNodeCount = -1;
	static int PreviousConnectionCount = -1;

	const int CurrentNodeCount = Graph.GetNodeCount();
	const int CurrentConnectionCount = static_cast<int>(Graph.GetConnections().size());

	// Check if the graph has updated
	if (CurrentNodeCount != PreviousNodeCount || CurrentConnectionCount != PreviousConnectionCount)
	{
		EulerianPath EulerPath(&Graph);
		Eulerianity EulerType = Eulerianity::notEulerian;

		// Run the EulerianPath algorithm
		std::vector<Node*> Trail = EulerPath.FindPath(EulerType);

		// If a path is found, have the agent follow it
		if (!Trail.empty())
		{
			UpdateAgentPath(Trail);
		}
		else
		{
			std::vector<FVector2D> emptyPath;
			PathFollow.SetPath(emptyPath);
		}

		PreviousNodeCount = CurrentNodeCount;
		PreviousConnectionCount = CurrentConnectionCount;
	}
}

void ALevel_GraphTheory::UpdateAgentPath(std::vector<Node*> const& Trail)
{
	std::vector<FVector2D> path{};

	// convert Node vector to positions vector
	for (Node* NodePtr : Trail)
	{
		if (NodePtr)
		{
			path.push_back(NodePtr->GetPosition());
		}
	}

	PathFollow.SetPath(path);
	if (path.size() > 0)
	{
		Agent->SetPosition(path[0]);
	}
}




