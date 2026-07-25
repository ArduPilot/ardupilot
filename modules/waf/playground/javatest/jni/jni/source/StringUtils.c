#include "StringUtils.h"
#include <string.h>
#include <ctype.h>

JNIEXPORT jboolean JNICALL Java_StringUtils_isAlpha(JNIEnv *env, jclass clazz,
        jstring jStr)
{
    jboolean ret = JNI_TRUE;
    char *sp = NULL, *s = NULL;

    if (!jStr)
        return JNI_FALSE;

    s = (char*)(*env)->GetStringUTFChars(env, jStr, 0);
    sp = s + strlen(s);
    if (sp <= s)
        ret = JNI_FALSE;
    do
    {
        if (!isalpha(*(--sp)))
            ret = JNI_FALSE;
    }
    while (sp > s);

    (*env)->ReleaseStringUTFChars(env, jStr, s);
    return ret;
}

JNIEXPORT jboolean JNICALL Java_StringUtils_isEmpty(JNIEnv *env, jclass clazz,
        jstring jStr)
{
    jboolean ret = JNI_TRUE;
    char *sp = NULL, *s = NULL;

    if (!jStr)
        return JNI_TRUE;

    s = (char*)(*env)->GetStringUTFChars(env, jStr, 0);
    sp = s + strlen(s);
    if (sp <= s)
        ret = JNI_TRUE;
    do
    {
        if (!isspace(*(--sp)))
            ret = JNI_FALSE;
    }
    while (sp > s);

    (*env)->ReleaseStringUTFChars(env, jStr, s);
    return ret;
}
