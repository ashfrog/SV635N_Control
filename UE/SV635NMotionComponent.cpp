#include "SV635NMotionComponent.h"

#include "Common/UdpSocketBuilder.h"
#include "Dom/JsonObject.h"
#include "HAL/PlatformTime.h"
#include "IPAddress.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

USV635NMotionComponent::USV635NMotionComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void USV635NMotionComponent::BeginPlay()
{
    Super::BeginPlay();
    FIPv4Address Address;
    if (!FIPv4Address::Parse(Host, Address) || Port < 1024 || Port > 65535)
    {
        LastStatus = TEXT("Invalid bridge IPv4 address or port");
        return;
    }
    ServerAddress = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    ServerAddress->SetIp(Address.Value);
    ServerAddress->SetPort(Port);
    Socket = FUdpSocketBuilder(TEXT("SV635N-UE"))
        .AsNonBlocking()
        .BoundToEndpoint(FIPv4Endpoint(FIPv4Address::Any, 0))
        .WithReceiveBufferSize(65536);
    LastStatus = Socket ? TEXT("Waiting for PC local permission") : TEXT("UDP socket creation failed");
}

void USV635NMotionComponent::SetMotorTargets(FVector MotorDegrees)
{
    if (!FMath::IsFinite(MotorDegrees.X) || !FMath::IsFinite(MotorDegrees.Y)
        || !FMath::IsFinite(MotorDegrees.Z))
    {
        SetMotorEnable(false);
        LastStatus = TEXT("Non-finite motor target; enable revoked");
        return;
    }
    Targets = MotorDegrees;
}

void USV635NMotionComponent::SetMotorEnable(bool Enable)
{
    WantEnable = Enable;
    if (!Enable && !Session.IsEmpty())
    {
        SendCommand(false);
    }
    HelloElapsed = 1; // Discover the PC session promptly when the user enables.
}

void USV635NMotionComponent::SendJson(const TSharedRef<FJsonObject>& Object)
{
    if (!Socket || !ServerAddress.IsValid()) return;
    FString Text;
    const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Text);
    FJsonSerializer::Serialize(Object, Writer);
    const FTCHARToUTF8 Bytes(*Text);
    int32 Sent = 0;
    if (!Socket->SendTo(reinterpret_cast<const uint8*>(Bytes.Get()), Bytes.Length(), Sent, *ServerAddress)
        || Sent != Bytes.Length())
    {
        LastStatus = TEXT("UDP send failed; PC heartbeat will stop motion");
    }
}

void USV635NMotionComponent::SendHello()
{
    const TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
    Object->SetStringField(TEXT("type"), TEXT("hello"));
    SendJson(Object);
}

void USV635NMotionComponent::SendCommand(bool Enable)
{
    if (Session.IsEmpty()) return;
    const TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
    Object->SetStringField(TEXT("type"), TEXT("command"));
    Object->SetStringField(TEXT("session"), Session);
    Object->SetNumberField(TEXT("seq"), static_cast<double>(Sequence++));
    Object->SetBoolField(TEXT("enable"), Enable);
    if (Enable)
    {
        TArray<TSharedPtr<FJsonValue>> Values;
        Values.Add(MakeShared<FJsonValueNumber>(Targets.X));
        Values.Add(MakeShared<FJsonValueNumber>(Targets.Y));
        Values.Add(MakeShared<FJsonValueNumber>(Targets.Z));
        Object->SetArrayField(TEXT("targets_deg"), Values);
    }
    SendJson(Object);
}

void USV635NMotionComponent::ReceiveFeedback()
{
    // Bound work per game frame; network frames never become queued motor moves.
    for (int32 Count = 0; Count < 16; ++Count)
    {
        uint32 Pending = 0;
        if (!Socket->HasPendingData(Pending)) break;
        TArray<uint8> Buffer;
        Buffer.SetNumUninitialized(65536);
        int32 Read = 0;
        const TSharedRef<FInternetAddr> Sender = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
        if (!Socket->RecvFrom(Buffer.GetData(), Buffer.Num() - 1, Read, *Sender)) break;
        if (Sender->ToString(true) != ServerAddress->ToString(true)) continue;
        Buffer[Read] = 0;
        TSharedPtr<FJsonObject> Object;
        const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(
            UTF8_TO_TCHAR(reinterpret_cast<const ANSICHAR*>(Buffer.GetData())));
        if (!FJsonSerializer::Deserialize(Reader, Object) || !Object.IsValid()) continue;
        FString Type, ReceivedSession;
        if (!Object->TryGetStringField(TEXT("type"), Type) || Type != TEXT("state")
            || !Object->TryGetStringField(TEXT("session"), ReceivedSession)) continue;
        if (Session != ReceivedSession)
        {
            Session = ReceivedSession;
            Sequence = 0;
        }
        LastFeedbackTime = FPlatformTime::Seconds();
        FeedbackValid = true;
        Object->TryGetNumberField(TEXT("timeout_s"), FeedbackTimeout);
        Object->TryGetBoolField(TEXT("enabled"), HardwareEnabled);
        Object->TryGetStringField(TEXT("phase"), LastStatus);
        bool Stopping = false;
        Object->TryGetBoolField(TEXT("stopping"), Stopping);
        if (Stopping || LastStatus == TEXT("closed")) WantEnable = false;
        FString Error;
        if (Object->TryGetStringField(TEXT("error"), Error)) LastStatus = Error;
        const TArray<TSharedPtr<FJsonValue>>* Orders = nullptr;
        if (Object->TryGetArrayField(TEXT("orders"), Orders))
        {
            MappedOrders.Reset();
            for (const auto& Value : *Orders) MappedOrders.Add(static_cast<int32>(Value->AsNumber()));
        }
        const TArray<TSharedPtr<FJsonValue>>* Axes = nullptr;
        if (Object->TryGetArrayField(TEXT("axes"), Axes) && Axes->Num() == 3)
        {
            for (int32 Axis = 0; Axis < 3; ++Axis)
            {
                const TSharedPtr<FJsonObject> Feedback = (*Axes)[Axis]->AsObject();
                double Degrees = 0;
                if (Feedback.IsValid() && Feedback->TryGetNumberField(TEXT("travel_degrees"), Degrees))
                    ActualMotorDegrees[Axis] = Degrees;
            }
        }
    }
}

void USV635NMotionComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                           FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    if (!Socket) return;
    ReceiveFeedback();
    if (!Session.IsEmpty() && FPlatformTime::Seconds() - LastFeedbackTime > FeedbackTimeout)
    {
        SetMotorEnable(false);
        FeedbackValid = false;
        Session.Reset();
        LastStatus = TEXT("Bridge feedback timeout; enable revoked");
    }
    HelloElapsed += DeltaTime;
    SendElapsed += DeltaTime;
    if (HelloElapsed >= .5f)
    {
        HelloElapsed = 0;
        SendHello();
    }
    if (WantEnable && !Session.IsEmpty() && SendElapsed >= 1.f / 30.f)
    {
        SendElapsed = 0;
        SendCommand(true);
    }
}

void USV635NMotionComponent::EndPlay(const EEndPlayReason::Type Reason)
{
    SetMotorEnable(false);
    if (Socket)
    {
        Socket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
        Socket = nullptr;
    }
    Super::EndPlay(Reason);
}
