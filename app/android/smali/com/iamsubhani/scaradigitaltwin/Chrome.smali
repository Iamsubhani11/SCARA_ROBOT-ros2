.class public Lcom/iamsubhani/scaradigitaltwin/Chrome;
.super Landroid/webkit/WebChromeClient;
.source "Chrome.java"

# lets the page use the camera (hand control) once the app has the Android permission

.method public constructor <init>()V
    .registers 1
    invoke-direct {p0}, Landroid/webkit/WebChromeClient;-><init>()V
    return-void
.end method

.method public onPermissionRequest(Landroid/webkit/PermissionRequest;)V
    .registers 3
    invoke-virtual {p1}, Landroid/webkit/PermissionRequest;->getResources()[Ljava/lang/String;
    move-result-object v0
    invoke-virtual {p1, v0}, Landroid/webkit/PermissionRequest;->grant([Ljava/lang/String;)V
    return-void
.end method
