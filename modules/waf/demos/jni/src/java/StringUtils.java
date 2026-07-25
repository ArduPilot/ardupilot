public final class StringUtils
{

    public static final String LIBRARY_NAME = "stringUtils";

    static
    {
        System.loadLibrary(LIBRARY_NAME);
    }

    private StringUtils()
    {
    }

    public static native boolean isAlpha(String string);

    public static native boolean isEmpty(String string);

    public static void main(String[] args)
    {
        System.out.println(StringUtils.isAlpha("sureIs"));
        System.out.println(StringUtils.isAlpha("nope!"));
        System.out.println(StringUtils.isEmpty("   "));
        System.out.println(StringUtils.isEmpty("nope"));
    }

}
