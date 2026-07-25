import pyexample as m


def test_example():
    obj = m.Example()
    obj.something = 42
    assert obj.something == 42
    assert obj.calculate() == 47  # with default argument
    assert obj.calculate(2) == 44
