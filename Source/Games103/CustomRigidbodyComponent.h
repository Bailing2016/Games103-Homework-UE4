#pragma once

#include "CoreMinimal.h"
#include "Kismet/GameplayStatics.h"
#include "Components/StaticMeshComponent.h"
#include "CustomRigidbodyComponent.generated.h"

struct FObstacleInfo
{
	FObstacleInfo(const FBox& B, const FVector& N, const FVector& P)
		: Bounds(B)
		, Normal(N)
		, Position(P)
	{
	}
	FBox Bounds;
	FVector Normal;
	FVector Position;
};

UCLASS(editinlinenew, meta = (BlueprintSpawnableComponent), ClassGroup = Physics)
class GAMES103_API UCustomRigidbodyComponent : public UStaticMeshComponent
{
	GENERATED_UCLASS_BODY()

public:

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	float Mass = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	float Restitution = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	float LinearDecay = 0.999f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	float AngularDecay = 0.98f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CustomRigidbody")
	float SubStepTime = 1.0f / 60;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	FName ObstacleTag = "";

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:

	FBox RigidbodyBounds;
	FPositionVertexBuffer* VertexBufferPtr = nullptr;
	float InvMass = 0.0f;
	FVector CenterOfMass = FVector(0.0f);
	FMatrix InvInertiaMatrixUnitMass;
	FMatrix InvInertiaMatrix;
	TArray<FObstacleInfo> ObstablesInfo;
	float RemainTime = 0.0f;
	// 苇莱尔积分使用的相关变量.
	FVector Position;
	FVector Acceleration;
	FVector LinearVelocity;
	FVector AngularVelocity;
	FQuat Rotation;

	void InitRigidbodyState();
	void CollectObstacleInfo();
	void CalculateRigidbodyParameters();
	void OnMassChanged();
	FVector GetForce() const;
	void UpdateRigidbodyState(const float DeltaTime);
	void PostUpdateRigidbodyState();
	void PerformRigidbodyCollision();
	void PerformRigidBodyCollision(const FObstacleInfo& Info, const FTransform& Trans, const FBox& CurRigidbodyBounds);

};
