import pyttsx3

def speak_number(number_to_say):
    # Initialize the TTS engine
    engine = pyttsx3.init()
    
    # Optional: Speed up or slow down the speech (default is usually around 200)
    engine.setProperty('rate', 150)
    
    # Format the single number as a string
    text = str(number_to_say)
    
    # Tell the engine what to say and block until it finishes
    engine.say(text)
    engine.runAndWait()

# Example: Speak a single number
speak_number(7)
