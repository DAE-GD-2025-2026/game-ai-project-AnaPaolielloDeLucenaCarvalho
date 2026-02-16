#include "Level_CombinedSteering.h"
#include "imgui.h"

ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();
	
	if (!AgentClass)
	{
		if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Red, TEXT("ERROR: AgentClass is empty! Please click Level_CombinedSteering in the Outliner and set it in the Details Panel."));
		return;
	}

	FVector SpawnLoc = FVector::ZeroVector;
	m_pMyAgent = GetWorld()->SpawnActor<ASteeringAgent>(AgentClass, SpawnLoc, FRotator::ZeroRotator);
		
	if (m_pMyAgent)
	{
		m_pSeek = std::make_unique<Seek>();
		m_pWander = std::make_unique<Wander>();

		m_pBlendedSteering = std::make_unique<BlendedSteering>(std::vector<BlendedSteering::WeightedBehavior>{
			{ m_pSeek.get(), 0.5f },   // 50% Seek
			{ m_pWander.get(), 0.5f }  // 50% Wander
		});

		m_pMyAgent->SetSteeringBehavior(m_pBlendedSteering.get());
	}
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();
}

void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	//UI
	{
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scroll wheel: zoom cam.");
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
	
		ImGui::Text("Combined Steering");
		ImGui::Spacing();
		ImGui::Spacing();
	
		ImGui::Checkbox("Debug Rendering", &CanDebugRender);
		
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

		if (m_pBlendedSteering)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek Weight", m_pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f, [this](float InVal) { m_pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander Weight", m_pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f, [this](float InVal) { m_pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
		}

		ImGui::End();
	}
#pragma endregion

	if (m_pMyAgent)
	{
		APlayerController* PC = GetWorld()->GetFirstPlayerController();
		if (PC && PC->IsInputKeyDown(EKeys::LeftMouseButton))
		{
			UseMouseTarget = true;
		}

		if (UseMouseTarget && m_pSeek)
		{
			m_pSeek->SetTarget(MouseTarget);
		}
		
		if (CanDebugRender)
		{
			DrawDebugCircle(GetWorld(), m_pMyAgent->GetActorLocation(), 50.f, 24, FColor::Yellow, false, -1.f, 0, 3.f, FVector(1,0,0), FVector(0,1,0), false);
		}
	}
}