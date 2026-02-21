/**************************************************************************
 * Boids simulation using raylib
 * Based on paper by Craig Reynolds
 * Boids, Background and Update
 * https://www.red3d.com/cwr/boids/
 * 
 * YouTube video of an example in processing using Java:
 * https://youtu.be/L5l_G5A3T1k?si=pzPyfOfOHGDN9UTN
 * by Programming Chaos
 * 
 * The basic flocking model consists of three simple steering behaviors 
 * which describe how an individual boid maneuvers based on the 
 * positions and velocities its nearby flockmates:
 *  - Separation: steer to avoid crowding local flockmates
 *  - Alignment: steer towards the average heading of local flockmates
 *  - Cohesion: steer to move toward the average position of local flockmates
 *************************************************************************/

#include "raylib.h"
#include "raymath.h"
#include <time.h>
#include <stdbool.h>

#define MAX_BOIDS 100 // maximum number of boids to generate
#define FPS 60
#define BOID_SIZE 16.0f // Size of the boid texture

const float SCREEN_WIDTH = 50 * BOID_SIZE;
const float SCREEN_HEIGHT = 50 * BOID_SIZE;

const float SEPARATION_RADIUS  = (2.0f * BOID_SIZE); // Radius for separation behavior
const float ALIGNMENT_RADIUS = (3.0f * BOID_SIZE);  // Radius for alignment behavior
const float COHESION_RADIUS = (4.0f * BOID_SIZE);   // Radius for cohesion

const float TARGET_DETECTION_RADIUS = (12.0f * BOID_SIZE);   // Radius for boids to detect enemy and head towards it
const float TARGET_AVOIDANCE_RADIUS = (5.0f * BOID_SIZE);  // Distance below which boids head away from enemy

const float SEPARATION_K = 2.5f;
const float COHESION_K = 1.0f;
const float ALIGNMENT_K = 1.0f;
const float TARGET_ATTRACT_K = 1.25f;
const float TARGET_REPEL_K = 5.25f;

const float MAX_BOID_SPEED = (float) (2.5 * FPS); // pixels per second
const float MIN_BOID_SPEED = (float) (1.0 * FPS);  // pixels per second
const float MAX_FORCE = 0.04f * FPS;

const Color BACKGROUND_COLOR = { 25, 25, 25, 255 };
const char * const BOID_TEXTURE_PATH = "src/resources/boid.png"; // Path to the boid texture

Texture2D boidTexture; // Load boid.png into a texture

RenderTexture2D sceneTexture; // Render texture for the scene (framebuffer)

typedef struct Boid {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
    Color color;
} Boid;

// Array to store boids
Boid boids[MAX_BOIDS] = {0};
int boidCount = 0;

bool targetEnabled = false;
bool drawTargetRadius = false;
Vector2 targetPosition = {0, 0};

Vector2 Vector2Rand() {
    // 1. Generate random X and Y components between -100 and 100
    Vector2 randVec = {
        (float)GetRandomValue(-100, 100),
        (float)GetRandomValue(-100, 100)
    };

    // 2. Handle the rare case where both are 0 to avoid division by zero
    if (randVec.x == 0 && randVec.y == 0) return (Vector2){ 1, 0 };

    // 3. Use raymath to normalize it to length 1.0
    return Vector2Normalize(randVec);
}

void UnloadResources() {
    UnloadTexture(boidTexture);
    UnloadRenderTexture(sceneTexture);
}

//Function to calculate distance between two boids accounting for screen wrapping
//A boid at x=1 and one at x=799 are actually only 2 pixels apart on a wrapping 800px screen
float GetWrappedDistance(Vector2 posA, Vector2 posB){
    float dx = posA.x - posB.x;
    float dy = posA.y - posB.y;

    // Wrap delta to the shortest path across the screen
    if (dx >  SCREEN_WIDTH  * 0.5f) dx -= SCREEN_WIDTH;
    if (dx < -SCREEN_WIDTH  * 0.5f) dx += SCREEN_WIDTH;
    if (dy >  SCREEN_HEIGHT * 0.5f) dy -= SCREEN_HEIGHT;
    if (dy < -SCREEN_HEIGHT * 0.5f) dy += SCREEN_HEIGHT;

    return sqrtf(dx * dx + dy * dy);
}

//Function to generate boids with random positions and directions
void GenerateBoids() {
    for (int i = 0; i < MAX_BOIDS; i++) {
        boids[i].position = (Vector2){ (float)GetRandomValue(0, (int)SCREEN_WIDTH), (float)GetRandomValue(0, (int)SCREEN_HEIGHT) };
        boids[i].velocity = Vector2Scale(Vector2Rand(), MAX_BOID_SPEED);
        boids[i].acceleration = (Vector2){ 0, 0 };
        boids[i].color = WHITE;
        boidCount++;
    }
}

// Update boid positions based on precomputed acceleration
void UpdateBoids(float deltaTime, Boid flock[], int numBoids) {
    for (int i = 0; i < numBoids; i++) {

        flock[i].velocity = Vector2Add(flock[i].velocity, flock[i].acceleration);
        flock[i].velocity = Vector2ClampValue(flock[i].velocity, MIN_BOID_SPEED, MAX_BOID_SPEED); // Limit speed
        flock[i].position = Vector2Add(flock[i].position, Vector2Scale(flock[i].velocity, deltaTime));
        
        flock[i].acceleration = (Vector2){ 0, 0 }; // Reset acceleration for next frame

        // Wrap the boids on screen (preserving fractional position)
        if (flock[i].position.x < 0) flock[i].position.x += SCREEN_WIDTH;
        if (flock[i].position.x >= SCREEN_WIDTH) flock[i].position.x -= SCREEN_WIDTH;
        if (flock[i].position.y < 0) flock[i].position.y += SCREEN_HEIGHT;
        if (flock[i].position.y >= SCREEN_HEIGHT) flock[i].position.y -= SCREEN_HEIGHT;
    }
}

// Steer away from nearby boids to avoid crowding (Reynolds: Separation)
Vector2 ComputeSeparation(Boid flock[], int numBoids, int boidIndex) {
    Vector2 sum = Vector2Zero();
    int neighbors = 0;

    for (int j = 0; j < numBoids; j++) {
        if (boidIndex == j) continue;
        float dist = GetWrappedDistance(flock[boidIndex].position, flock[j].position);
        if (dist < SEPARATION_RADIUS && dist > 0.0f) {
            Vector2 diff = Vector2Subtract(flock[boidIndex].position, flock[j].position);
            diff = Vector2Normalize(diff);
            diff = Vector2Scale(diff, 1.0f / dist); // Weight by inverse distance: closer boids push harder
            sum = Vector2Add(sum, diff);
            neighbors++;
        }
    }

    if (neighbors == 0) return Vector2Zero();

    Vector2 steer = Vector2Normalize(sum);
    steer = Vector2Scale(steer, MAX_BOID_SPEED);              // Desired velocity
    steer = Vector2Subtract(steer, flock[boidIndex].velocity); // Steering force
    steer = Vector2ClampValue(steer, 0.0f, MAX_FORCE);        // Clamp force
    steer = Vector2Scale(steer, SEPARATION_K);                // Scale by weight
    return steer;
}

// Steer toward the average heading of nearby boids (Reynolds: Alignment)
Vector2 ComputeAlignment(Boid flock[], int numBoids, int boidIndex) {
    Vector2 sumVelocity = Vector2Zero();
    int neighbors = 0;

    for (int j = 0; j < numBoids; j++) {
        if (boidIndex == j) continue;
        float dist = GetWrappedDistance(flock[boidIndex].position, flock[j].position);
        if (dist <= ALIGNMENT_RADIUS && dist > 0.0f) {
            sumVelocity = Vector2Add(sumVelocity, flock[j].velocity);
            neighbors++;
        }
    }

    if (neighbors == 0) return Vector2Zero();

    Vector2 steer = Vector2Scale(sumVelocity, 1.0f / (float)neighbors);   // Average velocity
    steer = Vector2Normalize(steer);                                      // Normalize direction
    steer = Vector2Scale(steer, MAX_BOID_SPEED);                          // Desired velocity
    steer = Vector2Subtract(steer, flock[boidIndex].velocity);            // Steering force
    steer = Vector2ClampValue(steer, 0.0f, MAX_FORCE);                    // Clamp force
    steer = Vector2Scale(steer, ALIGNMENT_K);                             // Scale by weight
    return steer;
}

// Steer toward the average position of nearby boids (Reynolds: Cohesion)
Vector2 ComputeCohesion(Boid flock[], int numBoids, int boidIndex) {
    Vector2 sumPosition = Vector2Zero();
    int neighbors = 0;

    for (int j = 0; j < numBoids; j++) {
        if (boidIndex == j) continue;
        float dist = GetWrappedDistance(flock[boidIndex].position, flock[j].position);
        if (dist <= COHESION_RADIUS && dist > 0.0f) {
            sumPosition = Vector2Add(sumPosition, flock[j].position);
            neighbors++;
        }
    }

    if (neighbors == 0) return Vector2Zero();

    Vector2 steer = Vector2Scale(sumPosition, 1.0f / (float)neighbors);    // Average position
    steer = Vector2Subtract(steer, flock[boidIndex].position);             // Direction TO center
    steer = Vector2Normalize(steer);                                       // Normalize direction
    steer = Vector2Scale(steer, MAX_BOID_SPEED);                           // Desired velocity
    steer = Vector2Subtract(steer, flock[boidIndex].velocity);             // Steering force
    steer = Vector2ClampValue(steer, 0.0f, MAX_FORCE);                     // Clamp force
    steer = Vector2Scale(steer, COHESION_K);                               // Scale by weight
    return steer;
}

// Steer toward or away from the target depending on distance.
// Also updates the boid's color to reflect its relationship to the target.
Vector2 ComputeTargetSteering(Boid *boid, Vector2 target) {
    float dist = GetWrappedDistance(boid->position, target);                // Account for screen wrapping
    Vector2 steer = Vector2Zero();

    if (dist < TARGET_AVOIDANCE_RADIUS) {
        Vector2 targetDirection = Vector2Subtract(boid->position, target);  // Direction away from target
        steer = Vector2Normalize(targetDirection);                       
        steer = Vector2Scale(steer, MAX_BOID_SPEED);                     
        steer = Vector2Subtract(steer, boid->velocity);                  // Steering force away from target
        steer = Vector2ClampValue(steer, 0.0f, MAX_FORCE);               // Clamp force
        steer = Vector2Scale(steer, TARGET_REPEL_K);                     // Scale by weight
        boid->color = RED;
    } else if (dist < TARGET_DETECTION_RADIUS) {
        Vector2 targetDirection = Vector2Subtract(target, boid->position); // Direction towards target
        steer = Vector2Normalize(targetDirection);
        steer = Vector2Scale(steer, MAX_BOID_SPEED);
        steer = Vector2Subtract(steer, boid->velocity);                  // Steering force toward target
        steer = Vector2ClampValue(steer, 0.0f, MAX_FORCE);               // Clamp force
        steer = Vector2Scale(steer, TARGET_ATTRACT_K);                   // Scale by weight
        boid->color = BLUE;
    } else {
        boid->color = WHITE;
    }

    return steer;
}

void SteerBoids(Boid flock[], int numBoids) {
    for (int i = 0; i < numBoids; i++) {
        flock[i].acceleration = Vector2Add(flock[i].acceleration, ComputeSeparation(flock, numBoids, i));
        flock[i].acceleration = Vector2Add(flock[i].acceleration, ComputeAlignment(flock, numBoids, i));
        flock[i].acceleration = Vector2Add(flock[i].acceleration, ComputeCohesion(flock, numBoids, i));

        if (targetEnabled) {
            flock[i].acceleration = Vector2Add(flock[i].acceleration, ComputeTargetSteering(&flock[i], targetPosition));
        } else {
            flock[i].color = WHITE;
        }
    }
}

// Passing boids array as argument to allow adding multiple flocks in future updates
void DrawBoids(Boid flock[], int numBoids) {
    for (int i = 0; i < numBoids; i++) {
        Vector2 boidOrigin = { (float)boidTexture.width * 0.5f, (float)boidTexture.height * 0.5f }; // Center the texture on the boid's position
        DrawTexturePro(
            boidTexture, 
            (Rectangle){0, 0, (float)boidTexture.width, (float)boidTexture.height}, 
            (Rectangle){flock[i].position.x, flock[i].position.y, (float)boidTexture.width, (float)boidTexture.height}, 
            boidOrigin,
            atan2f(flock[i].velocity.y, flock[i].velocity.x) * RAD2DEG, // Rotate texture to match direction
            flock[i].color); 
    }
}

void DrawTarget(Vector2 position, bool drawRadius){
    DrawCircleV(position, 6, RED);
    if (drawRadius){
        DrawCircleLinesV(position, TARGET_AVOIDANCE_RADIUS, RED);
        DrawCircleLinesV(position, TARGET_DETECTION_RADIUS, BLUE);
    }
}

int main()
{
    // Tell the window to use vsync and work on high DPI displays
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT);
    
    // Enable raylib debug logs
    //SetTraceLogLevel(LOG_DEBUG);
    
    InitWindow((int) SCREEN_WIDTH, (int) SCREEN_HEIGHT, "Boids Simulation");
    SetTargetFPS(FPS);

    SetRandomSeed((unsigned int)time(NULL)); // Seed the random number generator with the current time

    // We will use a single render texture for the scene
    sceneTexture = LoadRenderTexture((int) SCREEN_WIDTH, (int) SCREEN_HEIGHT); // Scene texture

    boidTexture = LoadTexture(BOID_TEXTURE_PATH);
    if (boidTexture.id == 0) {
        TraceLog(LOG_ERROR, "Failed to load texture: %s", BOID_TEXTURE_PATH);
        return -1;
    }
    SetTextureFilter(boidTexture, TEXTURE_FILTER_BILINEAR);

    GenerateBoids();

    while (!WindowShouldClose())
    {
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            targetEnabled = !targetEnabled;
        }
        if (targetEnabled) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                drawTargetRadius = !drawTargetRadius;
            }
        }

        targetPosition = GetMousePosition();
        
        SteerBoids(boids, boidCount);
        UpdateBoids(GetFrameTime(), boids, boidCount);
        
        // Render the scene to the scene texture
        BeginTextureMode(sceneTexture);
            ClearBackground(BACKGROUND_COLOR);
            DrawFPS(10, 10);
            DrawBoids(boids, boidCount);
            if (targetEnabled){
                DrawTarget(targetPosition, drawTargetRadius);
            }
        EndTextureMode();

        // Draw the final result to the screen
        BeginDrawing();
            ClearBackground(BACKGROUND_COLOR); // Clear the screen with black
                    
            // Draw the final scene without distortion
            DrawTexturePro(
                sceneTexture.texture, 
                (Rectangle){ 0, 0, (float)sceneTexture.texture.width, (float)-sceneTexture.texture.height},
                (Rectangle){ 0, 0, (float)SCREEN_WIDTH, (float)SCREEN_HEIGHT},
                (Vector2){ 0, 0 },
                0.0f,
                WHITE);
        EndDrawing();
    }

    // Unload resources
    UnloadResources();
    CloseWindow();

    return 0;
}
