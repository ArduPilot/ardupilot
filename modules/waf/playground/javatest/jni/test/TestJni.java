import org.testng.Assert;
import org.testng.annotations.Test;

public class TestJni {

	@Test
	public void testTrue() {
		Assert.assertEquals(true, StringUtils.isAlpha("myfootest"), "'myfootest' is alpha");
	}

	@Test
	public void testFalse() {
		Assert.assertEquals(false, StringUtils.isAlpha("my f00 t3$t"), "'my f00 t3$t' is not alpha");
	}

	@Test
	public void testIsEmpty() {
		Assert.assertEquals(false, StringUtils.isEmpty("emptyNOT"), "'emptyNOT' is not empty");
	}
}
