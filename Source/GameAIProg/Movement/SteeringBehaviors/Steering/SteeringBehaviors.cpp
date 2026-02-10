#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

// TODO: Do the Week01 assignment :^)

int g_DebugArrowSize = 150;
int g_DebugArrowLength = 200;

// ***************************************************************************************************************
// Seek

SteeringOutput Seek::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
    SteeringOutput steering{};

    steering.LinearVelocity = Target.Position - Agent.GetPosition();
    steering.LinearVelocity.Normalize();
    steering.LinearVelocity *= Agent.GetMaxLinearSpeed() * deltaT;

    if (Agent.GetDebugRenderingEnabled())
    {
        FVector2D dir{};
        double length{};

        steering.LinearVelocity.ToDirectionAndLength(dir, length);
        DrawDebugDirectionalArrow(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition() + dir* g_DebugArrowLength, 0), g_DebugArrowSize, FColor(255, 0, 0));

        Agent.GetLinearVelocity().ToDirectionAndLength(dir, length);
        DrawDebugDirectionalArrow(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition() + dir* g_DebugArrowLength, 0), g_DebugArrowSize, FColor(0, 255, 0));
    }

    return steering;
}

// ***************************************************************************************************************
// Flee

SteeringOutput Flee::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
    SteeringOutput steering{};

    steering.LinearVelocity = -1 * (Target.Position - Agent.GetPosition());
    steering.LinearVelocity.Normalize();
    steering.LinearVelocity *= Agent.GetMaxLinearSpeed() * deltaT;

    if (Agent.GetDebugRenderingEnabled())
    {
        FVector2D dir{};
        double length{};

        steering.LinearVelocity.ToDirectionAndLength(dir, length);
        DrawDebugDirectionalArrow(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition() + dir * g_DebugArrowLength, 0), g_DebugArrowSize, FColor(255, 0, 0));

        Agent.GetLinearVelocity().ToDirectionAndLength(dir, length);
        DrawDebugDirectionalArrow(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition() + dir * g_DebugArrowLength, 0), g_DebugArrowSize, FColor(0, 255, 0));
    }

    return steering;
}

// ***************************************************************************************************************
// Arrive

SteeringOutput Arrive::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
    SteeringOutput steering{};

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Size();

    if (distance < m_TargetRadius)
    {
        steering.LinearVelocity = FVector2D(0, 0);
    }
    else
    {
        FVector2D dir = toTarget.GetSafeNormal();

        if (distance < m_SlowRadius)
        {
            float factor = distance / m_SlowRadius;
            steering.LinearVelocity = dir * factor;
        }
        else
        {
            steering.LinearVelocity = dir;
        }

    }

    if (Agent.GetDebugRenderingEnabled())
    {
        //UE_LOG(LogTemp, Warning, TEXT("Distance: %f"), distance);
        //UE_LOG(LogTemp, Warning, TEXT("Speed: %f"), steering.LinearVelocity.Size());

        DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), m_SlowRadius, 16, FColor(0, 0, 255),false,-1.f, 0,0.0f, FVector(1.f, 0.f, 0.f), FVector(0.f, 1.f, 0.f),false);
        DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), m_TargetRadius, 16, FColor(255, 0, 0), false, -1.f, 0, 0.0f, FVector(1.f, 0.f, 0.f), FVector(0.f, 1.f, 0.f), false);

        FVector2D dir{};
        double length{};

        steering.LinearVelocity.ToDirectionAndLength(dir, length);
        DrawDebugDirectionalArrow(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition() + dir * g_DebugArrowLength, 0), g_DebugArrowSize, FColor(255, 0, 0));

        Agent.GetLinearVelocity().ToDirectionAndLength(dir, length);
        DrawDebugDirectionalArrow(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition() + dir * g_DebugArrowLength, 0), g_DebugArrowSize, FColor(0, 255, 0));
    }

    return steering;
}

// ***************************************************************************************************************
// Face

SteeringOutput Face::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
    SteeringOutput steering{};

    // Direction to target
    FVector2D toTarget = Target.Position - Agent.GetPosition();
    if (!toTarget.IsNearlyZero())
    {
        float currentAngle = FMath::DegreesToRadians(Agent.GetRotation());

        float targetAngle = FMath::Atan2(toTarget.Y, toTarget.X);
        float angleDiff = FMath::FindDeltaAngleRadians(currentAngle, targetAngle);

        float absAngle = FMath::Abs(angleDiff);
        if (absAngle < 0.1)
        {
            steering.AngularVelocity = 0.f;
        }
        else
        {
            steering.AngularVelocity = Agent.GetMaxAngularSpeed() * FMath::Sign(-angleDiff) * deltaT;
        }
    }

    if (Agent.GetDebugRenderingEnabled())
    {
        FVector agentPos3D(Agent.GetPosition(), 0.f);
        
        // Direction to target (red)
        FVector2D dirNorm = toTarget.GetSafeNormal();
        DrawDebugDirectionalArrow(Agent.GetWorld(), agentPos3D, FVector(Agent.GetPosition() + dirNorm * g_DebugArrowLength, 0.f), g_DebugArrowSize, FColor::Red);
    }

    return steering;
}

// ***************************************************************************************************************
// Persuit

SteeringOutput Pursuit::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
    SteeringOutput steering{};

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    const float distance = toTarget.Size();
    const float timeToTarget{ distance / Agent.GetMaxLinearSpeed() };

    steering.LinearVelocity = (Target.Position + Target.LinearVelocity * timeToTarget) - Agent.GetPosition();
    steering.LinearVelocity.Normalize();
    steering.LinearVelocity *= Agent.GetMaxLinearSpeed() * deltaT;

    if (Agent.GetDebugRenderingEnabled())
    {
        FVector2D dir{};
        double length{};

        steering.LinearVelocity.ToDirectionAndLength(dir, length);
        DrawDebugDirectionalArrow(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition() + dir * g_DebugArrowLength, 0), g_DebugArrowSize, FColor(255, 0, 0));

        // the location of the target in the future
        DrawDebugSphere(Agent.GetWorld(), FVector(Target.Position + Target.LinearVelocity * timeToTarget, 0), 20.f, 16, FColor(200, 200, 0));
    }

    return steering;
}

// ***************************************************************************************************************
// Evade

SteeringOutput Evade::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
    SteeringOutput steering{};

    FVector2D toTarget = Target.Position - Agent.GetPosition();
    const float distance = toTarget.Size();

    // if it is outside of the radius -> do not respond
    if (distance > m_Radius)
    {
        steering.IsValid = false;
    }
    else
    {
        steering = Pursuit::CalculateSteering(deltaT, Agent);
        steering.LinearVelocity *= -1;
    }

    if (Agent.GetDebugRenderingEnabled())
    {
        FVector2D dir{};
        double length{};

        steering.LinearVelocity.ToDirectionAndLength(dir, length);
        DrawDebugDirectionalArrow(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition() + dir * g_DebugArrowLength, 0), g_DebugArrowSize, FColor(0, 255, 0));

        DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition(), 0), m_Radius, 16, FColor(0, 0, 255), false, -1.f, 0, 0.0f, FVector(1.f, 0.f, 0.f), FVector(0.f, 1.f, 0.f), false);
    }

    return steering;
}

// ***************************************************************************************************************
// Wander

SteeringOutput Wander::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
    // calculate random angle
    m_WanderAngle += (((rand() / (double)RAND_MAX) * 2 * m_MaxAngleChange) - m_MaxAngleChange);

    FVector2D dir{};
    double length{};

    Agent.GetLinearVelocity().ToDirectionAndLength(dir, length);

    const FVector2D targetRadiusCenter{ Agent.GetPosition() + (dir * m_OffsetDistance) };
    const float target_x = targetRadiusCenter.X + m_Radius * cos(m_WanderAngle);
    const float target_y = targetRadiusCenter.Y + m_Radius * sin(m_WanderAngle);
    const FVector2D wanderTarget = { target_x, target_y };
    Target.Position = wanderTarget;

    SteeringOutput steering{Seek::CalculateSteering(deltaT, Agent)};

    if (Agent.GetDebugRenderingEnabled())
    {
        DrawDebugCircle(Agent.GetWorld(), FVector(targetRadiusCenter, 0), m_Radius, 16, FColor(0, 0, 255), false, -1.f, 0, 0.0f, FVector(1.f, 0.f, 0.f), FVector(0.f, 1.f, 0.f), false);
        DrawDebugPoint(Agent.GetWorld(), FVector(wanderTarget, 0), 20, FColor(0, 0, 0));
    }

    return steering;
}
