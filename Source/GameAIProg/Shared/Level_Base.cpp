#include "Level_Base.h"

// Sets default values
ALevel_Base::ALevel_Base()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_Base::BeginPlay()
{
	Super::BeginPlay();

	// Set ImGui defaults
	FImGuiModule::Get().GetProperties().SetInputEnabled(true);
	FImGuiModule::Get().GetProperties().SetMouseInputShared(true);
	FImGuiModule::Get().GetProperties().SetKeyboardNavigationEnabled(false);
	FImGuiModule::Get().GetProperties().SetKeyboardInputShared(true);
	FImGuiModule::Get().GetProperties().SetGamepadInputShared(true);
	FImGuiModule::Get().GetProperties().SetGamepadNavigationEnabled(false);

	// Spawn our trimworld
	TrimWorld = GetWorld()->SpawnActor<AWorldTrimVolume>(FVector{0,0,0}, FRotator::ZeroRotator);
}

// Called every frame
void ALevel_Base::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Update viewport data
	GetWorld()->GetGameViewport()->GetViewportSize(ViewportSize);
	WindowSize = {MenuWidth, static_cast<float>(ViewportSize.Y) - 20};
	WindowPos = {static_cast<float>(ViewportSize.X) - MenuWidth - 10, 10};

	//Render Target
	if (VisualizeMouseTarget)
		DrawDebugCylinder(GetWorld(), FVector{ MouseTarget.Position, 0 }, FVector{ MouseTarget.Position, 50 }, 10, 16, FColor::Red);
		//->DrawSolidCircle(MouseTarget.Position, 0.3f, { 0.f,0.f }, { 1.f,0.f,0.f },-0.8f);
}

