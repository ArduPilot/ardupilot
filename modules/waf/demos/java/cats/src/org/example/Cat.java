
package org.example;

import org.example.Animal;

class Cat extends Animal {

    public String sound() {
            // Use base class just to be able to trigger possible rebuilds based on base class change
            String base = super.sound();
            return "Meow!";
    }

}

