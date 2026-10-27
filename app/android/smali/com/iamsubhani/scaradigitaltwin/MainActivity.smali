.class public Lcom/iamsubhani/scaradigitaltwin/MainActivity;
.super Landroid/app/Activity;
.source "MainActivity.java"

# SCARA Digital Twin v3: a full-screen WebView showing assets/ui/index.html.
# The whole app (3-D view, joysticks, connection) lives in app/ui (HTML + JavaScript).
# Native extras for the page (window.ScaraNative, see Bridge): speech recognition for voice
# commands, and the camera permission for hand control (getUserMedia in the page).

.field private web:Landroid/webkit/WebView;

.method public constructor <init>()V
    .registers 1
    invoke-direct {p0}, Landroid/app/Activity;-><init>()V
    return-void
.end method

.method protected onCreate(Landroid/os/Bundle;)V
    .registers 6
    invoke-super {p0, p1}, Landroid/app/Activity;->onCreate(Landroid/os/Bundle;)V

    # keep the screen on while the app is open (it is a robot pendant)
    invoke-virtual {p0}, Landroid/app/Activity;->getWindow()Landroid/view/Window;
    move-result-object v0
    const/16 v1, 0x80
    invoke-virtual {v0, v1}, Landroid/view/Window;->addFlags(I)V

    new-instance v0, Landroid/webkit/WebView;
    invoke-direct {v0, p0}, Landroid/webkit/WebView;-><init>(Landroid/content/Context;)V
    iput-object v0, p0, Lcom/iamsubhani/scaradigitaltwin/MainActivity;->web:Landroid/webkit/WebView;

    invoke-virtual {v0}, Landroid/webkit/WebView;->getSettings()Landroid/webkit/WebSettings;
    move-result-object v1
    const/4 v2, 0x1
    invoke-virtual {v1, v2}, Landroid/webkit/WebSettings;->setJavaScriptEnabled(Z)V
    invoke-virtual {v1, v2}, Landroid/webkit/WebSettings;->setDomStorageEnabled(Z)V
    invoke-virtual {v1, v2}, Landroid/webkit/WebSettings;->setAllowFileAccess(Z)V
    invoke-virtual {v1, v2}, Landroid/webkit/WebSettings;->setAllowFileAccessFromFileURLs(Z)V
    const/4 v3, 0x0
    invoke-virtual {v1, v3}, Landroid/webkit/WebSettings;->setMediaPlaybackRequiresUserGesture(Z)V
    invoke-virtual {v1, v3}, Landroid/webkit/WebSettings;->setSupportZoom(Z)V

    # big system text is kept, but capped at 115 % so the pendant layout still fits the screen
    invoke-virtual {v1}, Landroid/webkit/WebSettings;->getTextZoom()I
    move-result v3
    const/16 v2, 0x73
    if-le v3, v2, :zoom_ok
    invoke-virtual {v1, v2}, Landroid/webkit/WebSettings;->setTextZoom(I)V
    :zoom_ok

    # tidy edges: no overscroll glow, the page draws its own background
    const/4 v3, 0x2
    invoke-virtual {v0, v3}, Landroid/view/View;->setOverScrollMode(I)V
    const v3, -0x1c1815
    invoke-virtual {v0, v3}, Landroid/webkit/WebView;->setBackgroundColor(I)V

    # native extras (voice, camera); the app still runs if anything here fails
    :try_start
    new-instance v1, Lcom/iamsubhani/scaradigitaltwin/Chrome;
    invoke-direct {v1}, Lcom/iamsubhani/scaradigitaltwin/Chrome;-><init>()V
    invoke-virtual {v0, v1}, Landroid/webkit/WebView;->setWebChromeClient(Landroid/webkit/WebChromeClient;)V
    new-instance v1, Lcom/iamsubhani/scaradigitaltwin/Bridge;
    invoke-direct {v1, p0}, Lcom/iamsubhani/scaradigitaltwin/Bridge;-><init>(Lcom/iamsubhani/scaradigitaltwin/MainActivity;)V
    const-string v3, "ScaraNative"
    invoke-virtual {v0, v1, v3}, Landroid/webkit/WebView;->addJavascriptInterface(Ljava/lang/Object;Ljava/lang/String;)V
    :try_end
    .catch Ljava/lang/Throwable; {:try_start .. :try_end} :extras_failed
    goto :load

    :extras_failed
    move-exception v1

    :load
    const-string v1, "file:///android_asset/ui/index.html"
    invoke-virtual {v0, v1}, Landroid/webkit/WebView;->loadUrl(Ljava/lang/String;)V
    invoke-virtual {p0, v0}, Landroid/app/Activity;->setContentView(Landroid/view/View;)V
    # keys of a USB / Bluetooth keyboard and gamepad go to the page
    const/4 v1, 0x1
    invoke-virtual {v0, v1}, Landroid/view/View;->setFocusable(Z)V
    invoke-virtual {v0, v1}, Landroid/view/View;->setFocusableInTouchMode(Z)V
    invoke-virtual {v0}, Landroid/view/View;->requestFocus()Z
    return-void
.end method

# run a line of JavaScript in the page
.method public js(Ljava/lang/String;)V
    .registers 4
    iget-object v0, p0, Lcom/iamsubhani/scaradigitaltwin/MainActivity;->web:Landroid/webkit/WebView;
    if-eqz v0, :done
    const/4 v1, 0x0
    invoke-virtual {v0, p1, v1}, Landroid/webkit/WebView;->evaluateJavascript(Ljava/lang/String;Landroid/webkit/ValueCallback;)V
    :done
    return-void
.end method

# called on the UI thread (Task): open the phone's speech recogniser
.method public startVoice()V
    .registers 5
    :try_start
    new-instance v0, Landroid/content/Intent;
    const-string v1, "android.speech.action.RECOGNIZE_SPEECH"
    invoke-direct {v0, v1}, Landroid/content/Intent;-><init>(Ljava/lang/String;)V
    const-string v1, "android.speech.extra.LANGUAGE_MODEL"
    const-string v2, "free_form"
    invoke-virtual {v0, v1, v2}, Landroid/content/Intent;->putExtra(Ljava/lang/String;Ljava/lang/String;)Landroid/content/Intent;
    const-string v1, "android.speech.extra.PROMPT"
    const-string v2, "Say a command, e.g. red, then blue"
    invoke-virtual {v0, v1, v2}, Landroid/content/Intent;->putExtra(Ljava/lang/String;Ljava/lang/String;)Landroid/content/Intent;
    const-string v1, "android.speech.extra.MAX_RESULTS"
    const/4 v2, 0x3
    invoke-virtual {v0, v1, v2}, Landroid/content/Intent;->putExtra(Ljava/lang/String;I)Landroid/content/Intent;
    const/16 v1, 0x7
    invoke-virtual {p0, v0, v1}, Landroid/app/Activity;->startActivityForResult(Landroid/content/Intent;I)V
    :try_end
    .catch Ljava/lang/Throwable; {:try_start .. :try_end} :no_recogniser
    return-void

    :no_recogniser
    move-exception v0
    const-string v1, "window.onNativeVoice && window.onNativeVoice(null, 'no-recognizer')"
    invoke-virtual {p0, v1}, Lcom/iamsubhani/scaradigitaltwin/MainActivity;->js(Ljava/lang/String;)V
    return-void
.end method

# called on the UI thread (Task): ask for the camera (hand control)
.method public askCamera()V
    .registers 4
    const/4 v0, 0x1
    new-array v0, v0, [Ljava/lang/String;
    const/4 v1, 0x0
    const-string v2, "android.permission.CAMERA"
    aput-object v2, v0, v1
    const/16 v1, 0x8
    invoke-virtual {p0, v0, v1}, Landroid/app/Activity;->requestPermissions([Ljava/lang/String;I)V
    return-void
.end method

.method public onRequestPermissionsResult(I[Ljava/lang/String;[I)V
    .registers 7
    const-string v0, "false"
    array-length v1, p3
    if-lez v1, :send
    const/4 v1, 0x0
    aget v1, p3, v1
    if-nez v1, :send
    const-string v0, "true"
    :send
    new-instance v1, Ljava/lang/StringBuilder;
    invoke-direct {v1}, Ljava/lang/StringBuilder;-><init>()V
    const-string v2, "window.onNativePermission && window.onNativePermission("
    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;
    invoke-virtual {v1, v0}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;
    const-string v2, ")"
    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;
    invoke-virtual {v1}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;
    move-result-object v1
    invoke-virtual {p0, v1}, Lcom/iamsubhani/scaradigitaltwin/MainActivity;->js(Ljava/lang/String;)V
    return-void
.end method

# the speech recogniser's answer -> window.onNativeVoice("text") (or null)
.method protected onActivityResult(IILandroid/content/Intent;)V
    .registers 8
    invoke-super {p0, p1, p2, p3}, Landroid/app/Activity;->onActivityResult(IILandroid/content/Intent;)V
    const/16 v0, 0x7
    if-ne p1, v0, :done
    const-string v0, "null"
    const/4 v1, -0x1
    if-ne p2, v1, :send
    if-eqz p3, :send
    const-string v1, "android.speech.extra.RESULTS"
    invoke-virtual {p3, v1}, Landroid/content/Intent;->getStringArrayListExtra(Ljava/lang/String;)Ljava/util/ArrayList;
    move-result-object v1
    if-eqz v1, :send
    invoke-virtual {v1}, Ljava/util/ArrayList;->size()I
    move-result v2
    if-lez v2, :send
    const/4 v2, 0x0
    invoke-virtual {v1, v2}, Ljava/util/ArrayList;->get(I)Ljava/lang/Object;
    move-result-object v1
    check-cast v1, Ljava/lang/String;
    invoke-static {v1}, Lorg/json/JSONObject;->quote(Ljava/lang/String;)Ljava/lang/String;
    move-result-object v0
    :send
    new-instance v1, Ljava/lang/StringBuilder;
    invoke-direct {v1}, Ljava/lang/StringBuilder;-><init>()V
    const-string v2, "window.onNativeVoice && window.onNativeVoice("
    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;
    invoke-virtual {v1, v0}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;
    const-string v2, ")"
    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;
    invoke-virtual {v1}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;
    move-result-object v1
    invoke-virtual {p0, v1}, Lcom/iamsubhani/scaradigitaltwin/MainActivity;->js(Ljava/lang/String;)V
    :done
    return-void
.end method

.method public onBackPressed()V
    .registers 3
    # back closes the connect sheet / toast inside the page first, then leaves the app
    iget-object v0, p0, Lcom/iamsubhani/scaradigitaltwin/MainActivity;->web:Landroid/webkit/WebView;
    if-eqz v0, :leave
    invoke-virtual {v0}, Landroid/webkit/WebView;->canGoBack()Z
    move-result v1
    if-eqz v1, :leave
    invoke-virtual {v0}, Landroid/webkit/WebView;->goBack()V
    return-void
    :leave
    invoke-super {p0}, Landroid/app/Activity;->onBackPressed()V
    return-void
.end method

.method protected onPause()V
    .registers 3
    invoke-super {p0}, Landroid/app/Activity;->onPause()V
    # tell the page (it stops the hand camera)
    const-string v0, "window.onNativePause && window.onNativePause()"
    invoke-virtual {p0, v0}, Lcom/iamsubhani/scaradigitaltwin/MainActivity;->js(Ljava/lang/String;)V
    return-void
.end method

.method protected onDestroy()V
    .registers 2
    iget-object v0, p0, Lcom/iamsubhani/scaradigitaltwin/MainActivity;->web:Landroid/webkit/WebView;
    if-eqz v0, :done
    invoke-virtual {v0}, Landroid/webkit/WebView;->destroy()V
    :done
    invoke-super {p0}, Landroid/app/Activity;->onDestroy()V
    return-void
.end method
