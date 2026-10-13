# android

This hwdef.dat builds ArduPilot SITL for Android (arm64) with the
Android NDK.

![Android robot](android-robot.svg)

In order to be called from JNI, each vehicle is linked as a shared
object. Its entry point is `ardupilot_main`, which takes the same
arguments as `main` and does not return.

## Building

The NDK compilers must be in the `PATH`:

```sh
export PATH=$ANDROID_NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/bin:$PATH
./waf configure --board android
./waf copter
```

The result is `build/android/bin/arducopter`. Android only packages
native libraries named `lib*.so`, so copy it into the app as
`libarducopter.so`.

## Using it from an app

`ardupilot_main` is a C function, so the app calls it through a small
JNI function of its own. On the Kotlin side, assuming that the JNI
library is `sitl_jni`:

```kotlin
object Sitl {
    init { System.loadLibrary("sitl_jni") }
    external fun run(workDir: String): Int
}
```

And in `libsitl_jni.so`, which is linked against `libarducopter.so`:

```c
int ardupilot_main(int argc, char *const argv[]);

JNIEXPORT jint JNICALL
Java_com_example_Sitl_run(JNIEnv *env, jobject thiz, jstring work_dir)
{
    // SITL keeps eeprom.bin, logs/ and terrain/ in the current directory,
    // which for an app is "/" unless changed
    const char *dir = (*env)->GetStringUTFChars(env, work_dir, NULL);
    chdir(dir);
    (*env)->ReleaseStringUTFChars(env, work_dir, dir);

    char *const argv[] = {"arducopter", "--model", "quad", NULL};
    return ardupilot_main(3, argv);
}
```

Run it in a process of its own, such as a service declared with
`android:process`. `ardupilot_main` does not return, so stopping the
simulator requires killing that process. A reboot also ends it: SITL
reboots by calling `execv` on its own executable and aborts if that
fails, which happens in an Android app.
