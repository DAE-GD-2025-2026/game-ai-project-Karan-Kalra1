#include "Level_CombinedSteering.h"

#include "imgui.h"


// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_CombinedSteering::BeginPlay()
{
		Super::BeginPlay();

		// Main agent
		pAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 0,0,90 }, FRotator::ZeroRotator);

		// Wanderer agent (to evade)
		pWanderer = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 200,200,90 }, FRotator::ZeroRotator);

		// Behaviors
		pSeek = new Seek();
		pWander = new Wander();
		pEvade = new Evade();
		pEvade->SetEvadeDistance(200.f);
		

		// Blended (Seek + Wander)
		std::vector<BlendedSteering::WeightedBehavior> behaviors
		{
			{pSeek, 0.5f},
			{pWander, 0.5f}
		};

		pBlended = new BlendedSteering(behaviors);

		// Priority: Evade overrides Blended
		std::vector<ISteeringBehavior*> priorityBehaviors
		{
			pEvade,
			pWander
		};

		pPriority = new PrioritySteering(priorityBehaviors);

		// Wanderer only wanders
		pWanderer->SetSteeringBehavior(pBlended);

		pAgent->SetSteeringBehavior(pPriority);
	

}

void ALevel_CombinedSteering::BeginDestroy()
{
	
	
		delete pPriority;
		delete pBlended;
		delete pSeek;
		delete pWander;
		delete pEvade;

		Super::BeginDestroy();
	
}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
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
		ImGui::Spacing();
	
		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();
	
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
   // TODO: Handle the debug rendering of your agents here :)
			pAgent->SetDebugRenderingEnabled(CanDebugRender);
			pWanderer->SetDebugRenderingEnabled(CanDebugRender);
		}
		

		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
				TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
				[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}
		
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Spacing();
	
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();


		 ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
			 pBlended->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
		 	[this](float InVal) { pBlended->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
		//
		 ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
		 pBlended->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
		 [this](float InVal) { pBlended->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
	
		//End
		ImGui::End();

	}
#pragma endregion
	
	// Combined Steering Update
 // TODO: implement handling mouse click input for seek
			
			pSeek->SetTarget(MouseTarget);
			
			
	
 // 
 // TODO: implement Make sure to also evade the wanderer

			FTargetData Target;
			Target.Position = pWanderer->GetPosition();
			Target.Orientation = pWanderer->GetRotation();
			Target.LinearVelocity = pWanderer->GetLinearVelocity();
			Target.AngularVelocity = pWanderer->GetAngularVelocity();

			pEvade->SetTarget(Target);

}
