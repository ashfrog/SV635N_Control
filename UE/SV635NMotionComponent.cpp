#include "SV635NMotionComponent.h"
#include "Common/UdpSocketBuilder.h"
#include "Dom/JsonObject.h"
#include "HAL/PlatformTime.h"
#include "IPAddress.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "Misc/Guid.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

namespace { constexpr double MaxSequence = 9007199254740991.; }

USV635NMotionComponent::USV635NMotionComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = false;
}

void USV635NMotionComponent::BeginPlay()
{
    Super::BeginPlay();
    FIPv4Address Address;
    if (!FIPv4Address::Parse(Host, Address) || Port < 1 || Port > 65535
        || !FMath::IsFinite(SendRateHz) || SendRateHz < 10 || SendRateHz > 60
        || !FMath::IsFinite(PoseInputTimeout) || PoseInputTimeout < .1f || PoseInputTimeout > .5f)
    { LastError = TEXT("Invalid endpoint or timing configuration"); return; }
    ServerAddress = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    ServerAddress->SetIp(Address.Value); ServerAddress->SetPort(Port);
    Socket = FUdpSocketBuilder(TEXT("SV635N-UE-v1")).AsNonBlocking()
        .BoundToEndpoint(FIPv4Endpoint(FIPv4Address::Any, 0)).WithReceiveBufferSize(65536);
    LastStatus = Socket ? TEXT("Observing backend; operator arm required") : TEXT("UDP socket creation failed");
}

void USV635NMotionComponent::SetPlatformPose(float HeaveMillimetres, float PitchDegrees, float RollDegrees)
{
    if (!FMath::IsFinite(HeaveMillimetres) || !FMath::IsFinite(PitchDegrees) || !FMath::IsFinite(RollDegrees))
    { FailSafe(TEXT("Non-finite platform pose")); return; }
    Pose = FVector(HeaveMillimetres, PitchDegrees, RollDegrees);
    LastPoseTime = FPlatformTime::Seconds();
}

bool USV635NMotionComponent::ArmPlatform(bool PhysicalNeutralConfirmed)
{
    const double Now = FPlatformTime::Seconds();
    if (!Socket || Ending || Pending.IsValid() || !Session.IsEmpty() || !RunId.IsEmpty() || WantEnable
        || !PhysicalNeutralConfirmed || !FeedbackValid || Now - LastFeedbackTime > FeedbackTimeout
        || !PlatformConfigured || CalibrationId.IsEmpty() || CalibrationId != BackendCalibrationId
        || Orders.Num() != 3 || Orders != MappedOrders || Phase != TEXT("idle")
        || LastPoseTime < 0 || Now - LastPoseTime > PoseInputTimeout)
    { LastError = TEXT("Arm requires fresh pose/feedback, matching calibration, idle backend and confirmed physical neutral"); return false; }
    WantEnable = true; LastError.Reset();
    const auto Object = Request(TEXT("hello"));
    Object->SetBoolField(TEXT("claim"), true); Reliable(Object);
    return true; // Asynchronous; this is not a servo-on acknowledgement.
}

void USV635NMotionComponent::StopPlatform()
{
    WantEnable = false;
    StreamRequests.Reset(); // Late errors from the stopped run must not revoke a later arm.
    if (!Socket || Session.IsEmpty() || RunId.IsEmpty()) return;
    if (Pending.IsValid() && PendingKind == TEXT("disable")) return;
    const auto Object = Request(TEXT("disable"));
    Object->SetStringField(TEXT("run_id"), RunId);
    if (Pending.IsValid())
    {
        // Preserve the enable request identity; stop again when its late ACK arrives.
        for (int32 Count = 0; Count < 3; ++Count) SendJson(Object);
    }
    else Reliable(Object);
}

void USV635NMotionComponent::FailSafe(const FString& Reason)
{ LastError = Reason; StopPlatform(); }

TSharedRef<FJsonObject> USV635NMotionComponent::Request(const FString& Kind)
{
    const auto Object = MakeShared<FJsonObject>();
    Object->SetNumberField(TEXT("v"), 1); Object->SetStringField(TEXT("type"), Kind);
    Object->SetStringField(TEXT("id"), FGuid::NewGuid().ToString(EGuidFormats::Digits));
    if (!AuthKey.IsEmpty()) Object->SetStringField(TEXT("auth_key"), AuthKey);
    if (!Session.IsEmpty()) Object->SetStringField(TEXT("session"), Session);
    if (Kind == TEXT("enable") || Kind == TEXT("release"))
        Object->SetNumberField(TEXT("control_seq"), static_cast<double>(++ControlSequence));
    return Object;
}

void USV635NMotionComponent::SendJson(const TSharedRef<FJsonObject>& Object)
{
    if (!Socket || !ServerAddress.IsValid()) return;
    FString Text; const auto Writer = TJsonWriterFactory<>::Create(&Text);
    if (!FJsonSerializer::Serialize(Object, Writer)) return;
    const FTCHARToUTF8 Bytes(*Text); int32 Sent = 0;
    if (Bytes.Length() > 8192 || !Socket->SendTo(reinterpret_cast<const uint8*>(Bytes.Get()),
        Bytes.Length(), Sent, *ServerAddress) || Sent != Bytes.Length())
        LastError = TEXT("UDP send failed; backend watchdog remains authoritative");
}

void USV635NMotionComponent::Reliable(const TSharedRef<FJsonObject>& Object)
{
    Pending = Object;
    Object->TryGetStringField(TEXT("id"), PendingId); Object->TryGetStringField(TEXT("type"), PendingKind);
    Attempts = 1; NextRetry = FPlatformTime::Seconds() + .12; SendJson(Object);
}

void USV635NMotionComponent::Observe()
{
    const auto Object = Request(TEXT("hello")); Object->SetBoolField(TEXT("claim"), false);
    Object->TryGetStringField(TEXT("id"), ObserveId); SendJson(Object);
}

void USV635NMotionComponent::SendStream(const FString& Kind)
{
    if (Sequence >= MaxSequence) { FailSafe(TEXT("Sequence exhausted; new session required")); return; }
    const auto Object = Request(Kind);
    Object->SetStringField(TEXT("run_id"), RunId);
    Object->SetNumberField(TEXT("seq"), static_cast<double>(Sequence++));
    if (Kind == TEXT("pose"))
    {
        const auto Values = MakeShared<FJsonObject>();
        Values->SetNumberField(TEXT("heave_mm"), Pose.X); Values->SetNumberField(TEXT("pitch_deg"), Pose.Y);
        Values->SetNumberField(TEXT("roll_deg"), Pose.Z); Object->SetObjectField(TEXT("pose"), Values);
    }
    FString Id; Object->TryGetStringField(TEXT("id"), Id);
    StreamRequests.Add(Id, FPlatformTime::Seconds()); SendJson(Object);
    // Do not retry old poses; the next current pose replaces them.
}

bool USV635NMotionComponent::ApplyState(const TSharedPtr<FJsonObject>& Object)
{
    FString Instance; double Serial = -1; const TSharedPtr<FJsonObject>* State = nullptr;
    if (!Object->TryGetStringField(TEXT("server_id"), Instance) || Instance != ServerId
        || !Object->TryGetNumberField(TEXT("state_serial"), Serial) || !FMath::IsFinite(Serial)
        || Serial < 0 || Serial > MaxSequence || Serial != FMath::FloorToDouble(Serial)
        || Serial <= StateSerial || !Object->TryGetObjectField(TEXT("state"), State) || !State->IsValid()) return false;
    FString ReceivedPhase; bool Enabled = false;
    if (!(*State)->TryGetStringField(TEXT("phase"), ReceivedPhase)
        || !(*State)->TryGetBoolField(TEXT("enabled"), Enabled)) return false;
    StateSerial = static_cast<int64>(Serial); Phase = ReceivedPhase; HardwareEnabled = Enabled;
    LastFeedbackTime = FPlatformTime::Seconds(); FeedbackValid = true;
    (*State)->TryGetStringField(TEXT("message"), LastStatus);
    const TSharedPtr<FJsonObject>* Platform = nullptr;
    if ((*State)->TryGetObjectField(TEXT("platform"), Platform) && Platform->IsValid())
    {
        (*Platform)->TryGetBoolField(TEXT("configured"), PlatformConfigured);
        (*Platform)->TryGetStringField(TEXT("calibration_id"), BackendCalibrationId);
        const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
        if ((*Platform)->TryGetArrayField(TEXT("orders"), Values))
        {
            MappedOrders.Reset();
            for (const auto& Value : *Values)
            {
                double Order = 0;
                if (!Value.IsValid() || !Value->TryGetNumber(Order) || !FMath::IsFinite(Order)
                    || Order < 1 || Order > MAX_int32 || Order != FMath::FloorToDouble(Order))
                { MappedOrders.Reset(); break; }
                MappedOrders.Add(static_cast<int32>(Order));
            }
        }
    }
    FString StateRun; (*State)->TryGetStringField(TEXT("run_id"), StateRun);
    FString Mode;
    (*State)->TryGetStringField(TEXT("control_mode"), Mode);
    if (RunId.IsEmpty() && PendingKind == TEXT("enable") && !Session.IsEmpty()
        && Mode == TEXT("platform") && (Phase == TEXT("enabling") || Phase == TEXT("enabled"))) RunId = StateRun;
    const TArray<TSharedPtr<FJsonValue>>* Axes = nullptr;
    if ((*State)->TryGetArrayField(TEXT("axes"), Axes))
    {
        for (const auto& Value : *Axes)
        {
            const TSharedPtr<FJsonObject>* Axis = nullptr; double Order = 0, Degrees = 0;
            if (!Value.IsValid() || !Value->TryGetObject(Axis) || !Axis->IsValid()
                || !(*Axis)->TryGetNumberField(TEXT("order"), Order)
                || !(*Axis)->TryGetNumberField(TEXT("travel_degrees"), Degrees) || !FMath::IsFinite(Degrees)) continue;
            for (int32 Index = 0; Index < Orders.Num() && Index < 3; ++Index)
                if (Order == Orders[Index]) ActualMotorDegrees[Index] = Degrees;
        }
    }
    if (WantEnable && !RunId.IsEmpty() && (Phase == TEXT("fault") || Phase == TEXT("stopping")
        || StateRun != RunId || Phase == TEXT("idle"))) FailSafe(TEXT("Backend stopped or run changed; rearm required"));
    return true;
}

void USV635NMotionComponent::CompleteRequest(const TSharedPtr<FJsonObject>& Object)
{
    const FString Kind = PendingKind; Pending.Reset(); PendingId.Reset(); PendingKind.Reset();
    bool Ok = false;
    if (!Object->TryGetBoolField(TEXT("ok"), Ok) || !Ok)
    {
        FString Error; Object->TryGetStringField(TEXT("error"), Error);
        FailSafe(Error.IsEmpty() ? TEXT("Control request rejected") : Error); return;
    }
    if (Kind == TEXT("hello"))
    {
        double Control = -1;
        if (!Object->TryGetStringField(TEXT("session"), Session) || Session.IsEmpty()
            || !Object->TryGetNumberField(TEXT("control_seq"), Control) || !FMath::IsFinite(Control)
            || Control < -1 || Control >= MaxSequence || Control != FMath::FloorToDouble(Control))
        { FailSafe(TEXT("Invalid control session")); return; }
        ControlSequence = static_cast<int64>(Control);
        if (!WantEnable) return;
        if (Phase != TEXT("idle") || !PlatformConfigured || CalibrationId != BackendCalibrationId || Orders != MappedOrders)
        { FailSafe(TEXT("Configuration/state changed during claim")); return; }
        const auto Enable = Request(TEXT("enable"));
        Enable->SetStringField(TEXT("mode"), TEXT("platform"));
        Enable->SetStringField(TEXT("calibration_id"), CalibrationId); Enable->SetBoolField(TEXT("reference_confirmed"), true);
        TArray<TSharedPtr<FJsonValue>> Values;
        for (int32 Order : Orders) Values.Add(MakeShared<FJsonValueNumber>(Order));
        Enable->SetArrayField(TEXT("orders"), Values); Sequence = 0; Reliable(Enable);
    }
    else if (Kind == TEXT("enable"))
    {
        FString ReturnedRun;
        if (!Object->TryGetStringField(TEXT("run_id"), ReturnedRun) || ReturnedRun.IsEmpty())
        { FailSafe(TEXT("Enable outcome unknown; heartbeats revoked")); return; }
        RunId = ReturnedRun;
        if (!WantEnable) StopPlatform();
    }
    else if (Kind == TEXT("release")) Session.Reset();
}

void USV635NMotionComponent::ReceiveFeedback()
{
    for (int32 Count = 0; Count < 32; ++Count)
    {
        uint32 PendingBytes = 0; if (!Socket->HasPendingData(PendingBytes)) break;
        TArray<uint8> Buffer; Buffer.SetNumUninitialized(65536); int32 Read = 0;
        const auto Sender = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
        if (!Socket->RecvFrom(Buffer.GetData(), Buffer.Num() - 1, Read, *Sender)) break;
        if (Read <= 0 || Read > 8192 || Sender->ToString(true) != ServerAddress->ToString(true)) continue;
        Buffer[Read] = 0; TSharedPtr<FJsonObject> Object;
        const auto Reader = TJsonReaderFactory<>::Create(UTF8_TO_TCHAR(reinterpret_cast<const ANSICHAR*>(Buffer.GetData())));
        if (!FJsonSerializer::Deserialize(Reader, Object) || !Object.IsValid()) continue;
        FString Type, Id, Instance; double Version = 0;
        if (!Object->TryGetNumberField(TEXT("v"), Version) || Version != 1
            || !Object->TryGetStringField(TEXT("type"), Type)) continue;
        Object->TryGetStringField(TEXT("id"), Id); Object->TryGetStringField(TEXT("server_id"), Instance);
        if (Type == TEXT("shutdown") && !ServerId.IsEmpty() && Instance == ServerId)
        {
            FailSafe(TEXT("Backend shutting down")); FeedbackValid = false;
            ServerId.Reset(); Session.Reset(); RunId.Reset(); Pending.Reset(); PendingId.Reset(); PendingKind.Reset(); continue;
        }
        const bool IsPending = Type == TEXT("ack") && !PendingId.IsEmpty() && Id == PendingId;
        const bool IsObserver = Type == TEXT("ack") && !ObserveId.IsEmpty() && Id == ObserveId;
        const bool IsStream = Type == TEXT("ack") && StreamRequests.Contains(Id);
        if (Type != TEXT("state") && !IsPending && !IsObserver && !IsStream) continue;
        bool Ok = false; Object->TryGetBoolField(TEXT("ok"), Ok);
        if (IsObserver && Ok && !Instance.IsEmpty() && Instance != ServerId)
        {
            if (!ServerId.IsEmpty()) FailSafe(TEXT("Backend instance changed; explicit rearm required"));
            ServerId = Instance; StateSerial = -1; Session.Reset(); RunId.Reset();
            Pending.Reset(); PendingId.Reset(); PendingKind.Reset(); StreamRequests.Reset();
            PlatformConfigured = false; BackendCalibrationId.Reset(); MappedOrders.Reset();
        }
        if (!Instance.IsEmpty() && Instance != ServerId) continue;
        if (!ServerId.IsEmpty()) ApplyState(Object);
        if (IsPending && Pending.IsValid() && Id == PendingId) CompleteRequest(Object);
        else if (IsStream)
        {
            StreamRequests.Remove(Id);
            if (!Ok) { FString Error; Object->TryGetStringField(TEXT("error"), Error); FailSafe(Error); }
        }
        else if (IsObserver && !Ok) Object->TryGetStringField(TEXT("error"), LastError);
    }
}

void USV635NMotionComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                         FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    if (!Socket || Ending) return;
    const double Now = FPlatformTime::Seconds();
    // Revoke intent before handling a late enable ACK after a game hitch.
    if (WantEnable && (LastPoseTime < 0 || Now - LastPoseTime > PoseInputTimeout))
        FailSafe(TEXT("Game pose stream stale; explicit rearm required"));
    ReceiveFeedback();
    if (FeedbackValid && Now - LastFeedbackTime > FeedbackTimeout)
    { FeedbackValid = false; FailSafe(TEXT("Backend feedback stale; explicit rearm required")); }
    if (Pending.IsValid() && Now >= NextRetry)
    {
        if (Attempts >= 8)
        {
            Pending.Reset(); PendingId.Reset(); PendingKind.Reset();
            FailSafe(TEXT("Control ACK timeout; outcome unknown, watchdog will stop the run"));
        }
        else { ++Attempts; NextRetry = Now + .12; SendJson(Pending.ToSharedRef()); }
    }
    if (Now - LastObserveTime >= .2) { LastObserveTime = Now; Observe(); }
    if (WantEnable && FeedbackValid && !RunId.IsEmpty() && Now - LastSendTime >= 1. / SendRateHz)
    {
        LastSendTime = Now;
        if (Phase == TEXT("enabled")) SendStream(TEXT("pose"));
        else if (Phase == TEXT("enabling")) SendStream(TEXT("heartbeat"));
    }
    if (!WantEnable && !Pending.IsValid() && !Session.IsEmpty() && FeedbackValid
        && (Phase == TEXT("idle") || Phase == TEXT("fault")))
    { RunId.Reset(); Reliable(Request(TEXT("release"))); }
    for (auto It = StreamRequests.CreateIterator(); It; ++It)
        if (Now - It.Value() > 1.) It.RemoveCurrent();
}

void USV635NMotionComponent::EndPlay(const EEndPlayReason::Type Reason)
{
    Ending = true; StopPlatform();
    if (Socket)
    {
        if (Pending.IsValid() && PendingKind == TEXT("disable"))
            for (int32 Count = 0; Count < 3; ++Count) SendJson(Pending.ToSharedRef());
        Socket->Close(); ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket); Socket = nullptr;
    }
    // In-flight enable receives no more heartbeats; backend watchdog remains active.
    Super::EndPlay(Reason);
}
