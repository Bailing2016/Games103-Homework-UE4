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

	const AActor* Owner = GetOwner();
	check(Owner);
	Position = GetComponentTransform().TransformPosition(CenterOfMass);
	Acceleration = LinearVelocity = AngularVelocity = FVector(0);
	Rotation = GetComponentQuat();
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
	const FVector Force = Mass * Gravity - LinearDamping * LinearVelocity + GetForce();
	const FVector NewPosition = Position + LinearVelocity * DeltaTime + 0.5f * Acceleration * DeltaTime * DeltaTime;
	const FVector NewAccleration = Force * InvMass;
	const FVector NewLinearVelocity = LinearVelocity + 0.5f * (Acceleration + NewAccleration) * DeltaTime;
	Position = NewPosition;
	Acceleration = NewAccleration;
	LinearVelocity = NewLinearVelocity;

	Rotation += FQuat(AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z, 0.0f) * Rotation * (0.5f * DeltaTime);
	Rotation.Normalize();
	AngularVelocity -= AngularDamping * AngularVelocity * DeltaTime;
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
	if (!ObstacleBounds.Intersect(CurRigidbodyBounds))
	{
		return;
	}

	int32 NumVerticesPosition = 0;
	FVector DeltaPosition(0.0f);
	FVector DeltaLinearVelocity(0.0f);
	FVector DeltaAngularVelocity(0.0f);
	int32 NumVerticesVelocity = 0;
	const FMatrix TransMatrix = Trans.ToMatrixNoScale();
	const FMatrix InvInertiaMatrixWS = TransMatrix * InvInertiaMatrix * TransMatrix.GetTransposed();
	const uint32 NumVertices = VertexBufferPtr->GetNumVertices();
	for (uint32 Index = 0; Index < NumVertices; Index++)
	{
		const FVector& VertexMS = VertexBufferPtr->VertexPosition(Index);
		const FVector& VertexWS = Trans.TransformPosition(VertexMS);
		const FVector Ra = Trans.TransformVector(VertexMS - CenterOfMass);
		const FVector Va = LinearVelocity + FVector::CrossProduct(AngularVelocity, Ra);
		const float Phi = FVector::DotProduct(VertexWS - ObstaclePosition, ObstacleNormal);
		const bool IsCollision = Phi < 0.0f;
		if (IsCollision)
		{
			DeltaPosition -= Phi * ObstacleNormal;
			NumVerticesPosition++;
			if (FVector::DotProduct(Va, ObstacleNormal) < 0.0f)
			{
				const float ImpulseMagNum = -(1.0f + Restitution) * FVector::DotProduct(Va, ObstacleNormal);
				const FVector InvIRa = InvInertiaMatrixWS.TransformVector(FVector::CrossProduct(Ra, ObstacleNormal));
				const float ImpulseMagDen = InvMass + FVector::DotProduct(ObstacleNormal, FVector::CrossProduct(InvIRa, Ra));
				const float ImpulseMag = ImpulseMagNum / ImpulseMagDen;
				DeltaLinearVelocity += ImpulseMag * ObstacleNormal * InvMass;
				DeltaAngularVelocity += ImpulseMag * InvInertiaMatrixWS.TransformVector(FVector::CrossProduct(Ra, ObstacleNormal));
				NumVerticesVelocity++;
			}
		}
	}
	Position += NumVerticesPosition > 0 ? DeltaPosition / NumVerticesPosition : FVector(0.0f);
	const float RecipeNumVertices = NumVerticesVelocity > 0 ? 1.0f / NumVerticesVelocity : 0.0f;
	LinearVelocity += DeltaLinearVelocity * RecipeNumVertices;
	AngularVelocity += DeltaAngularVelocity * RecipeNumVertices;
}

void UCustomRigidbodyComponent::Reset(const FVector& NewPosition, const FQuat& NewRotation)
{
	Position = NewPosition;
	Acceleration = LinearVelocity = AngularVelocity = FVector(0);
	Rotation = NewRotation;
	PostUpdateRigidbodyState();
}

void UCustomRigidbodyComponent::ApplyVelocity(const FVector& NewVelocity)
{
	LinearVelocity = NewVelocity;
}


