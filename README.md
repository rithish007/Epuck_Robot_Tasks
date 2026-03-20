# Epuck_Robot_Tasks
Obstacle avoidance and Object following tasks in a controlled environment.
The robot earned the nickname “Potato” 🥔. The object it had to follow was red, so naturally it became “Tomato” 🍅. And the arena? That was the garden.


🟦 Obstacle Avoidance (Potato explores the garden)
- IR-sensor-based wall detection
- state switching between explore and avoid
- directional turning based on left/right proximity
- bias logic to avoid getting stuck in tight spaces

🟩 Object Following (Potato chases Tomato)
- distance-based following using the ToF sensor
- track mode to approach or retreat safely
- search mode with rotational scanning when Tomato disappears
- fully autonomous behaviour using local sensing only

# Output
