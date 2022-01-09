#include "entt.hpp"
#include "CustomRigidbodyComponent.h"


UCustomRigidbodyComponent::UCustomRigidbodyComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UCustomRigidbodyComponent::BeginPlay()
{
	Super::BeginPlay();

	InitRigidbodyState();
	CollectObstacleInfo();
}

void UCustomRigidbodyComponent::InitRigidbodyState()
{
	FStaticMeshRenderData* RenderData = GetStaticMesh()->GetRenderData();
	RigidbodyBounds = RenderData->Bounds.GetBox();
	VertexBufferPtr = &(RenderData->LODResources[0].VertexBuffers.PositionVertexBuffer);
	CalculateRigidbodyParameters();
	OnMassChanged();
	Reset(GetComponentTransform().TransformPosition(CenterOfMass), GetComponentQuat(), false);
}

void UCustomRigidbodyComponent::CollectObstacleInfo()
{
	TArray<AActor*> ObstacleActors;
	UGameplayStatics::GetAllActorsWithTag(this, ObstacleTag, ObstacleActors);

	ObstablesInfo.Empty();
	for (const AActor* Actor : ObstacleActors)
	{
		const FBox BoundingBox = Actor->GetComponentsBoundingBox(false, true);
		if (BoundingBox.IsValid)
		{
			ObstablesInfo.Emplace(BoundingBox, Actor->GetTransform().GetUnitAxis(EAxis::Z), Actor->GetActorLocation());
		}
	}
}

void UCustomRigidbodyComponent::OnMassChanged()
{
	InvMass = Mass > 0 ? 1.0f / Mass : 0.0f;
	InvInertiaMatrix = InvInertiaMatrixUnitMass * InvMass;
}

void UCustomRigidbodyComponent::CalculateRigidbodyParameters()
{
	const uint32 NumVertices = VertexBufferPtr->GetNumVertices();
	check(NumVertices);
	FVector Center(0.0f);
	FVector InertiaAxis[] = {FVector(0), FVector(0), FVector(0)};
	for (uint32 Index = 0; Index < NumVertices; Index++)
	{
		const FVector Vertex = VertexBufferPtr->VertexPosition(Index);

		Center += Vertex;
		const float Diag = Vertex.SizeSquared();
		for (uint8 I = 0; I < 3; I++)
		{
			InertiaAxis[I][I] += Diag;
			for (uint8 J = 0; J < 3; J++)
			{
				InertiaAxis[I][J] -= Vertex[I]* Vertex[J];
			}
		}
	}
	const float RecipeNumVertices = 1.0f / NumVertices;
	CenterOfMass = Center * RecipeNumVertices;
	FMatrix InertiaMatrix;
	InertiaMatrix.SetIdentity();
	InertiaAxis[0] *= RecipeNumVertices;
	InertiaAxis[1] *= RecipeNumVertices;
	InertiaAxis[2] *= RecipeNumVertices;
	InertiaMatrix.SetAxes(&InertiaAxis[0], &InertiaAxis[1], &InertiaAxis[2]);
	InvInertiaMatrixUnitMass = InertiaMatrix.Inverse();
}

void UCustomRigidbodyComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bEnableSimulation)
	{
		return;
	}

	RemainTime += DeltaTime;
	while (RemainTime > SubStepTime)
	{
		UpdateRigidbodyState(SubStepTime);
		RemainTime -= SubStepTime;
	}

	PerformRigidbodyCollision();
	PostUpdateRigidbodyState();
}

FVector UCustomRigidbodyComponent::GetForce() const
{
	return FVector(0);
}

void UCustomRigidbodyComponent::UpdateRigidbodyState(const float DeltaTime)
{
	// 使用改进欧拉法, 具有二阶精度.
	const float HalfTime = DeltaTime * 0.5f;
	// 预报
	const FVector Force = Mass * Gravity + GetForce();
	const FVector NewPosition = Position + LinearVelocity * DeltaTime;
	const FVector Accleration = Force * InvMass - LinearDamping * LinearVelocity;
	const FVector NewLinearVelocity = LinearVelocity + Accleration * DeltaTime;
	// 矫正
	Position += (LinearVelocity + NewLinearVelocity) * HalfTime;
	const FVector NewAccleration = Force * InvMass - LinearDamping * NewLinearVelocity;
	LinearVelocity += (Accleration + NewAccleration) * HalfTime;

	// 更新角速度和旋转
	const FVector NewAngualerVelocity = AngularVelocity - AngularDamping * AngularVelocity;
	const FQuat DeltaRotation = FQuat(AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z, 0.0f) * Rotation * HalfTime;
	const FQuat NewRotation = (Rotation + DeltaRotation).GetNormalized();
	const FQuat NewDeltaRotation = FQuat(NewAngualerVelocity.X, NewAngualerVelocity.Y, NewAngualerVelocity.Z, 0.0f) * NewRotation * HalfTime;
	Rotation += (DeltaRotation + NewDeltaRotation) * 0.5f;
	Rotation.Normalize();
	AngularVelocity -= AngularDamping * (AngularVelocity + NewAngualerVelocity) * HalfTime;
}

void UCustomRigidbodyComponent::PostUpdateRigidbodyState() 
{
	AActor* Owner = GetOwner();
	Owner->SetActorLocation(Position + Rotation.RotateVector(-CenterOfMass));
	Owner->SetActorRotation(Rotation);
	GEngine->AddOnScreenDebugMessage(0, 3, FColor::Yellow, Position.ToString() + 
		"\n" + Rotation.ToString() +
		"\n" + Owner->GetActorRotation().ToString() +
		"\n" + AngularVelocity.ToString()
	);
}

void UCustomRigidbodyComponent::PerformRigidbodyCollision()
{
	FTransform Trans(Rotation, Position + Rotation.RotateVector(-CenterOfMass));
	const FBox CurRigidbodyBounds = RigidbodyBounds.TransformBy(Trans);
	for (const FObstacleInfo& Info : ObstablesInfo)
	{
		PerformRigidBodyCollision(Info, Trans, CurRigidbodyBounds);
	}
}

void UCustomRigidbodyComponent::PerformRigidBodyCollision(const FObstacleInfo& Info, const FTransform& Trans, const FBox& CurRigidbodyBounds)
{
	const auto [ObstacleBounds, ObstacleNormal, ObstaclePosition] = Info;
	// AABB 不相交，不可能有点和障碍物相交了.
	if (!ObstacleBounds.Intersect(CurRigidbodyBounds))
	{
		return;
	}

	float DeltaPosition = 0.0f;
	int32 NumVerticesPosition = 0;
	FVector DeltaVelocity(0.0f);
	int32 NumVerticesVelocity = 0;
	const uint32 NumVertices = VertexBufferPtr->GetNumVertices();
	for (uint32 Index = 0; Index < NumVertices; Index++)
	{
		const FVector& VertexMS = VertexBufferPtr->VertexPosition(Index);
		const FVector& VertexWS = Trans.TransformPosition(VertexMS);
		const FVector Ra = Trans.TransformVector(VertexMS - CenterOfMass);
		const FVector Va = LinearVelocity + (AngularVelocity ^ Ra);
		const float Phi = (VertexWS - ObstaclePosition) | ObstacleNormal;
		if (Phi < 0.0f)
		{
			DeltaPosition -= Phi;
			NumVerticesPosition++;
			if ((Va | ObstacleNormal) < 0.0f)
			{
				DeltaVelocity += Ra;
				NumVerticesVelocity++;
			}
		}
	}

	Position += NumVerticesPosition > 0 ? (DeltaPosition / NumVerticesPosition) * ObstacleNormal : FVector(0.0f);

	if (NumVerticesVelocity > 0)
	{
		// 计算碰撞后的速度
		const FMatrix TransMatrix = Trans.ToMatrixNoScale();
		const FMatrix InvInertiaMatrixWS = TransMatrix * InvInertiaMatrix * TransMatrix.GetTransposed();
		const FVector Ra = DeltaVelocity / NumVerticesVelocity;
		const FVector Va = LinearVelocity + (AngularVelocity ^ Ra);
		const float NormalSpeed = Va | ObstacleNormal;
		const FVector VaN = NormalSpeed * ObstacleNormal;
		const FVector VaT = Va - VaN;
		// 根据FMath::Max的定义，如果VaT.Size() = 0，那么结果应该是0，所以无需判断?
		const float A = FMath::Max(1 - Friction * (1 + Restitution) * VaN.Size() / VaT.Size(), 0.0f);
		// 这里有个trick, 就是速度越大恢复力效果越接近真实值.
		const FVector NewVaN = -Restitution * FMath::Clamp(FMath::Abs(NormalSpeed) * RestitutionDamping, 0.0f, 1.0f) * VaN;
		const FVector NewVaT = A * VaT;
		const FMatrix CrossProductMatrix = GetCrossProductMatrix(Ra);
		// todo FMatrix居然没有重载-运算符?
		const FMatrix K = FMatrix::Identity * InvMass + (CrossProductMatrix * InvInertiaMatrixWS * CrossProductMatrix * -1.0f);
		const FVector J = K.Inverse().TransformVector(NewVaN + NewVaT - Va);
		LinearVelocity += InvMass * J;
		AngularVelocity += InvInertiaMatrixWS.TransformVector(Ra ^ J);
	}
}

void UCustomRigidbodyComponent::Reset(const FVector& NewPosition, const FQuat& NewRotation, const bool bUpdateActorState)
{
	Position = NewPosition;
	LinearVelocity = AngularVelocity = FVector(0);
	Rotation = NewRotation;
	if (bUpdateActorState)
	{
		PostUpdateRigidbodyState();
	}
}

void UCustomRigidbodyComponent::ApplyVelocity(const FVector& NewVelocity)
{
	LinearVelocity = NewVelocity;
}


