#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SV635NMotionComponent.generated.h"

class FSocket;
class FInternetAddr;

UCLASS(ClassGroup=(Motion), meta=(BlueprintSpawnableComponent))
class USV635NMotionComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    USV635NMotionComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="SV635N")
    FString Host = TEXT("127.0.0.1");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="SV635N", meta=(ClampMin="1024", ClampMax="65535"))
    int32 Port = 5005;

    UPROPERTY(BlueprintReadOnly, Category="SV635N")
    bool HardwareEnabled = false;

    UPROPERTY(BlueprintReadOnly, Category="SV635N")
    bool FeedbackValid = false;

    UPROPERTY(BlueprintReadOnly, Category="SV635N")
    FString LastStatus;

    UPROPERTY(BlueprintReadOnly, Category="SV635N")
    FVector ActualMotorDegrees = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category="SV635N")
    TArray<int32> MappedOrders;

    // X/Y/Z map to the bridge's three orders, not platform pitch/roll/heave.
    UFUNCTION(BlueprintCallable, Category="SV635N")
    void SetMotorTargets(FVector MotorDegrees);

    UFUNCTION(BlueprintCallable, Category="SV635N")
    void SetMotorEnable(bool Enable);

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                               FActorComponentTickFunction* ThisTickFunction) override;

private:
    FSocket* Socket = nullptr;
    TSharedPtr<FInternetAddr> ServerAddress;
    FString Session;
    FVector Targets = FVector::ZeroVector;
    int64 Sequence = 0;
    bool WantEnable = false;
    float SendElapsed = 0;
    float HelloElapsed = 1;
    double LastFeedbackTime = 0;
    double FeedbackTimeout = .5;

    void SendHello();
    void SendCommand(bool Enable);
    void SendJson(const TSharedRef<class FJsonObject>& Object);
    void ReceiveFeedback();
};
