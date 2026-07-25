import org.testng.Assert;
import org.testng.annotations.Test;

public class TestNums {
	Nums myNumObj = new Nums();

	@Test
	public void testTrue() {
		myNumObj.setValue(10);

		boolean isBigger = myNumObj.isBiggerThanFive();
		Assert.assertEquals(true, isBigger, "10 should be bigger than 5");
	}

	@Test
	public void testFalse() {
		myNumObj.setValue(1);

		boolean isBigger = myNumObj.isBiggerThanFive();
		Assert.assertEquals(false, isBigger, "1 should be smaller than 5");
	}

	@Test
	public void testBoundary() {
		myNumObj.setValue(5);

		boolean isBigger = myNumObj.isBiggerThanFive();
		Assert.assertEquals(false, isBigger, "5 should not be bigger than 5");
	}
}

