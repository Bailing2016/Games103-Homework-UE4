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
	float Restitution = 0.1f;

	// 保证速度在很小的时候，恢复力不会导致抖动问题.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	float RestitutionDamping = 1e-3f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	float Friction = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	float LinearDamping = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	float AngularDamping = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CustomRigidbody")
	float SubStepTime = 1.0f / 60;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CustomRigidbody")
	FName ObstacleTag = "Obstacle";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CustomRigidbody")
	bool bEnableSimulation = true;

	UFUNCTION(BlueprintCallable, Category = "CustomRigidbody")
	void Reset(const FVector& NewPosition, const FQuat& NewRotation, const bool bUpdateActorState = true);

	UFUNCTION(BlueprintCallable, Category = "CustomRigidbody")
	void ApplyVelocity(const FVector& NewVelocity);

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
	// 状态
	FVector Position;
	FVector LinearVelocity;
	FQuat Rotation;
	FVector AngularVelocity;

	void InitRigidbodyState();
	void CollectObstacleInfo();
	void CalculateRigidbodyParameters();
	void OnMassChanged();
	FVector GetForce() const;
	void UpdateRigidbodyState(const float DeltaTime);
	void PostUpdateRigidbodyState();
	void PerformRigidbodyCollision();
	void PerformRigidBodyCollision(const FObstacleInfo& Info, const FTransform& Trans, const FBox& CurRigidbodyBounds);

	FORCEINLINE FMatrix GetCrossProductMatrix(const FVector& Vector)
	{
		FMatrix Matrix(FMatrix::Identity);
		Matrix.SetAxis(0, FVector(0, -Vector.Z, Vector.Y));
		Matrix.SetAxis(1, FVector(Vector.Z, 0, -Vector.X));
		Matrix.SetAxis(2, FVector(-Vector.Y, Vector.X, 0));
		return Matrix;
	}

};
