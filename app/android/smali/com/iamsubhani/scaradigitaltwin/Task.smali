.class public Lcom/iamsubhani/scaradigitaltwin/Task;
.super Ljava/lang/Object;
.source "Task.java"

# Runnable posted to the UI thread by Bridge: 1 = start speech recognition, 2 = ask for the camera
.implements Ljava/lang/Runnable;

.field private final act:Lcom/iamsubhani/scaradigitaltwin/MainActivity;
.field private final kind:I

.method public constructor <init>(Lcom/iamsubhani/scaradigitaltwin/MainActivity;I)V
    .registers 3
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V
    iput-object p1, p0, Lcom/iamsubhani/scaradigitaltwin/Task;->act:Lcom/iamsubhani/scaradigitaltwin/MainActivity;
    iput p2, p0, Lcom/iamsubhani/scaradigitaltwin/Task;->kind:I
    return-void
.end method

.method public run()V
    .registers 3
    iget-object v0, p0, Lcom/iamsubhani/scaradigitaltwin/Task;->act:Lcom/iamsubhani/scaradigitaltwin/MainActivity;
    iget v1, p0, Lcom/iamsubhani/scaradigitaltwin/Task;->kind:I
    packed-switch v1, :kinds
    return-void
    :voice
    invoke-virtual {v0}, Lcom/iamsubhani/scaradigitaltwin/MainActivity;->startVoice()V
    return-void
    :camera
    invoke-virtual {v0}, Lcom/iamsubhani/scaradigitaltwin/MainActivity;->askCamera()V
    return-void
    :kinds
    .packed-switch 0x1
        :voice
        :camera
    .end packed-switch
.end method
