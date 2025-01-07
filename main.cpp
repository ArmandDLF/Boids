#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include <stdexcept>
#include <SDL2/SDL.h>

#include "Boids.h"


void do_render() {
	// Reset to black screen
	SDL_SetRenderDrawColor(w.renderer, 0u, 0u, 0u, SDL_ALPHA_OPAQUE);
	SDL_RenderClear(w.renderer);
	
	// Render the scene
	for (Object<SDLColorType>* x : w.sceneObjects){
		x->draw();
	}

	SDL_RenderPresent(w.renderer);
}

void do_update() {
	// Update the scene
	if (!w.running) return;
	for (Object<SDLColorType>* x : w.sceneObjects){
		x->update();
	}
}

void init_scene(){
	// Empty scene 
}

int main(int argc, char ** argv)
{
	std::srand(time(NULL));

	int status;

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_EVENTS | SDL_INIT_TIMER) != 0) {
		throw std::runtime_error("Failed to initialize SDL");
	}

	w.window = SDL_CreateWindow("BOIDS",
			SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
			WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
	if (not w.window) {
		throw std::runtime_error("Failed to create SDL window");
	}

	// get the default renderer
	w.renderer = SDL_CreateRenderer(w.window, -1, 0);
	if (not w.renderer) {
		throw std::runtime_error("Failed to create SDL renderer");
	}

	init_scene();
	bool end = false;
	while (not end) {
		SDL_Event event;
		if (SDL_WaitEventTimeout(&event, 10)) {
			switch (event.type) {
			case SDL_WINDOWEVENT:
				switch (event.window.event) {
					case SDL_WINDOWEVENT_CLOSE:
						end = true;
						break;
					default:
						break;
				}
				break;
			case SDL_KEYDOWN:
				if (event.key.keysym.sym == SDLK_ESCAPE) {
					end = true;
				}
				if (event.key.keysym.sym == SDLK_BACKSPACE){
					// Delete all objects
					for (Object<SDLColorType>* x : w.sceneObjects){
						if (!x->obstacle){
							delete x;
						}
					}
						w.sceneObjects.clear();
					}
				if (event.key.keysym.sym == SDLK_SPACE){
					// Pause the simulation
					w.running = !w.running;

				}
				break;
			case SDL_KEYUP:
				break;
			case SDL_MOUSEBUTTONDOWN:					
				// Add a boid on left click where the mouse is
				if (event.button.button == SDL_BUTTON_LEFT){
					bool found = false;
					// Loop through objects and toggle selected if close enough
					for (Object<SDLColorType>* x : w.sceneObjects){
						if (distance((float)event.button.x, (float)event.button.y, x->x, x->y) < SELECT_DISTANCE){
							x->selected = !x->selected;
							found = true;
							break;
						}
					}
					if (!found) {
						auto color = std::make_tuple<Uint8, Uint8, Uint8, Uint8>(255,0,0,255);
						Boids * b =  new Boids(event.button.x, event.button.y, BOIDS_SIZE, color, 1., 0.);
						w.sceneObjects.push_back(b);
					}
				}
				// If right click all boids are deleted
				if (event.button.button == SDL_BUTTON_RIGHT){
					auto color = std::make_tuple<Uint8, Uint8, Uint8, Uint8>(0,255,0,255);
					Obstacles * obst =  new Obstacles(event.button.x, event.button.y, OBSTACLES_SIZE, color);
					w.sceneObjects.push_back(obst);
				}
				break;
			}
		} else {
			// Got time out or error
			char const * e = SDL_GetError();
			if (e != NULL) {
				if (strlen(e) != 0) {
					// Print and Ignore errors (MacOS has error)
					std::cout << "ERROR: " << e << std::endl;
					SDL_ClearError();
				}
			}

			do_update();
			do_render();
		}
	}

	SDL_DestroyRenderer(w.renderer);
	SDL_DestroyWindow(w.window);
	SDL_CloseAudio();
	SDL_Quit();
	return 0;
}
