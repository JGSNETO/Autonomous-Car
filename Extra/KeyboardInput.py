import keyboard

def readkeyboard():
    
    print("Press 'a' to trigger the event. Press 'esc' to exit.")

    # This will continuously check for key presses
    while True:
        if keyboard.is_pressed('a'):
            print("You pressed 'a'!")
            keyboard.wait('a')  # Waits until 'a' is released to avoid flooding
        elif keyboard.is_pressed('esc'):
            print("Exiting...")
            break

def main():
    readkeyboard()

if __name__ == '__main__':
    main()