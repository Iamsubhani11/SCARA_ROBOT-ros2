.class public Lcom/iamsubhani/scaradigitaltwin/Bridge;
.super Ljava/lang/Object;
.source "Bridge.java"

# window.ScaraNative in the page

.field private final act:Lcom/iamsubhani/scaradigitaltwin/MainActivity;

.method public constructor <init>(Lcom/iamsubhani/scaradigitaltwin/MainActivity;)V
    .registers 2
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V
    iput-object p1, p0, Lcom/iamsubhani/scaradigitaltwin/Bridge;->act:Lcom/iamsubhani/scaradigitaltwin/MainActivity;
    return-void
.end method

.method public version()Ljava/lang/String;
    .registers 2
    .annotation runtime Landroid/webkit/JavascriptInterface;
    .end annotation
    const-string v0, "3.0.0"
    return-object v0
.end method

.method public voiceAvailable()Z
    .registers 2
    .annotation runtime Landroid/webkit/JavascriptInterface;
    .end annotation
    const/4 v0, 0x1
    return v0
.end method

.method public startVoice()V
    .registers 4
    .annotation runtime Landroid/webkit/JavascriptInterface;
    .end annotation
    iget-object v0, p0, Lcom/iamsubhani/scaradigitaltwin/Bridge;->act:Lcom/iamsubhani/scaradigitaltwin/MainActivity;
    new-instance v1, Lcom/iamsubhani/scaradigitaltwin/Task;
    const/4 v2, 0x1
    invoke-direct {v1, v0, v2}, Lcom/iamsubhani/scaradigitaltwin/Task;-><init>(Lcom/iamsubhani/scaradigitaltwin/MainActivity;I)V
    invoke-virtual {v0, v1}, Landroid/app/Activity;->runOnUiThread(Ljava/lang/Runnable;)V
    return-void
.end method

.method public hasCamera()Z
    .registers 3
    .annotation runtime Landroid/webkit/JavascriptInterface;
    .end annotation
    iget-object v0, p0, Lcom/iamsubhani/scaradigitaltwin/Bridge;->act:Lcom/iamsubhani/scaradigitaltwin/MainActivity;
    const-string v1, "android.permission.CAMERA"
    invoke-virtual {v0, v1}, Landroid/app/Activity;->checkSelfPermission(Ljava/lang/String;)I
    move-result v0
    if-eqz v0, :yes                      # PERMISSION_GRANTED is 0
    const/4 v0, 0x0
    return v0
    :yes
    const/4 v0, 0x1
    return v0
.end method

.method public askCamera()V
    .registers 4
    .annotation runtime Landroid/webkit/JavascriptInterface;
    .end annotation
    iget-object v0, p0, Lcom/iamsubhani/scaradigitaltwin/Bridge;->act:Lcom/iamsubhani/scaradigitaltwin/MainActivity;
    new-instance v1, Lcom/iamsubhani/scaradigitaltwin/Task;
    const/4 v2, 0x2
    invoke-direct {v1, v0, v2}, Lcom/iamsubhani/scaradigitaltwin/Task;-><init>(Lcom/iamsubhani/scaradigitaltwin/MainActivity;I)V
    invoke-virtual {v0, v1}, Landroid/app/Activity;->runOnUiThread(Ljava/lang/Runnable;)V
    return-void
.end method
