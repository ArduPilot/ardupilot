public class Nums {
	private int value = 0;

	// Is bigger than 5
	public boolean isBiggerThanFive() {
		if (this.value > 5) {
			return true;
		} else {
			return false;
		}
	}

	// Set object value
	public void setValue(int value) {
		this.value = value;
	}
}
