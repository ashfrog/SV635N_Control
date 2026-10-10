#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SV635NMotionComponent.generated.h"

class FSocket;
class FInternetAddr;
class FJsonObject;

// UDP v1 platform client. Faults, game pauses and stale poses require explicit rearming.
UCLASS(ClassGroup=(Motion), meta=(BlueprintSpawnableComponent))
class USV635NMotionComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    USV635NMotionComponent();
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="SV635N|Connection")
    FString Host = TEXT("127.0.0.1");
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="SV635N|Connection", meta=(ClampMin="1", ClampMax="65535"))
    int32 Port = 5005;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="SV635N|Connection")
    FString AuthKey;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="SV635N|Calibration")
    FString CalibrationId;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="SV635N|Calibration")
    TArray<int32> Orders = {1, 2, 3};
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="SV635N|Timing", meta=(ClampMin="10", ClampMax="60"))
    float SendRateHz = 30.f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="SV635N|Timing", meta=(ClampMin="0.1", ClampMax="0.5"))
    float PoseInputTimeout = .2f;
    // Last observed hardware value. FeedbackValid must also be checked.
    UPROPERTY(BlueprintReadOnly, Category="SV635N|Feedback")
    bool HardwareEnabled = false;
    UPROPERTY(BlueprintReadOnly, Category="SV635N|Feedback")
    bool FeedbackValid = false;
    UPROPERTY(BlueprintReadOnly, Category="SV635N|Feedback")
    bool PlatformConfigured = false;
    UPROPERTY(BlueprintReadOnly, Category="SV635N|Feedback")
    FString LastStatus;
    UPROPERTY(BlueprintReadOnly, Category="SV635N|Feedback")
    FString LastError;
    UPROPERTY(BlueprintReadOnly, Category="SV635N|Feedback")
    FVector ActualMotorDegrees = FVector::ZeroVector;
    UPROPERTY(BlueprintReadOnly, Category="SV635N|Feedback")
    TArray<int32> MappedOrders;

    // Millimetres, not Unreal centimetres. Pitch front-up, roll right-up.
    // Call every game update, including a stationary pose.
    UFUNCTION(BlueprintCallable, Category="SV635N|Control")
    void SetPlatformPose(float HeaveMillimetres, float PitchDegrees, float RollDegrees);
    UFUNCTION(BlueprintCallable, Category="SV635N|Control")
    bool ArmPlatform(bool PhysicalNeutralConfirmed);
    UFUNCTION(BlueprintCallable, Category="SV635N|Control")
    void StopPlatform();
protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                               FActorComponentTickFunction* ThisTickFunction) override;
private:
    FSocket* Socket = nullptr;
    TSharedPtr<FInternetAddr> ServerAddress;
    FString Session, ServerId, RunId, Phase, BackendCalibrationId;
    FString ObserveId, PendingId, PendingKind;
    TSharedPtr<FJsonObject> Pending;
    TMap<FString, double> StreamRequests;
    FVector Pose = FVector::ZeroVector;
    int64 Sequence = 0, ControlSequence = -1, StateSerial = -1;
    bool WantEnable = false, Ending = false;
    double LastPoseTime = -1, LastFeedbackTime = -1;
    double LastSendTime = 0, LastObserveTime = 0, NextRetry = 0;
    int32 Attempts = 0;
    double FeedbackTimeout = .35;
    TSharedRef<FJsonObject> Request(const FString& Kind);
    void SendJson(const TSharedRef<FJsonObject>& Object);
    void Reliable(const TSharedRef<FJsonObject>& Object);
    void Observe();
    void SendStream(const FString& Kind);
    void ReceiveFeedback();
    bool ApplyState(const TSharedPtr<FJsonObject>& Object);
    void CompleteRequest(const TSharedPtr<FJsonObject>& Object);
    void FailSafe(const FString& Reason);
};
