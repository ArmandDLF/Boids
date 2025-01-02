
#include <SDL2/SDL.h>
#include "it_s_work.h"

#include <iostream>
#include <random>
#include <vector>
#include <cmath>

int const WIDTH = 800;
int const HEIGHT = 600;
float const PI = 3.1415927; // TODO: better PI

struct global_t {
	SDL_Window * window = NULL;
	SDL_Renderer * renderer = NULL;

	// random
	std::random_device rd;
	std::default_random_engine eng;
	std::uniform_real_distribution<float> rand;

};

global_t g;

void paint_it_s_work(int ox, int oy, int scale = 20) {
	SDL_SetRenderDrawColor(g.renderer, 0u, 0u, 0u, SDL_ALPHA_OPAQUE);
	for (int j = 0; j < px::height; ++j) {
		for (int i = 0; i < px::width; ++i) {
			if (px::header_data[j*px::width+i] == 0) {
				SDL_Rect r = { i*scale+ox, j*scale+oy, 20, 20 };
				SDL_RenderFillRect(g.renderer, &r);
			}
		}
	}
}

void do_render() {
	SDL_SetRenderDrawColor(g.renderer, 255u, 255u, 255u, SDL_ALPHA_OPAQUE);
	SDL_RenderClear(g.renderer);

	paint_it_s_work(0, 0, 20);

	SDL_RenderPresent(g.renderer);
}

void do_update() {

}

int main(int argc, char ** argv)
{


	int status;

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_EVENTS | SDL_INIT_TIMER) != 0) {
		return 1;
	}

	g.window = SDL_CreateWindow("Ant War",
			SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
			WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
	if (not g.window) {
		return 1;
	}

	// get the default renderer
	g.renderer = SDL_CreateRenderer(g.window, -1, 0);
	if (not g.renderer) {
		return 1;
	}

	bool end = false;
	while (not end) {
		SDL_Event event;
		if (SDL_WaitEventTimeout(&event, 20)) {
			switch (event.type) {
			case SDL_WINDOWEVENT:
				switch (event.window.event) {
					case SDL_WINDOWEVENT_CLOSE:
						end = true;
						break;
					case SDL_WINDOWEVENT_SIZE_CHANGED:
						// Should never happen
						break;
					default:
						break;
				}
				break;
			case SDL_KEYDOWN:
				if (event.key.keysym.sym == SDLK_ESCAPE) {
					end = true;
				}
				break;
			case SDL_KEYUP:
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

	SDL_DestroyRenderer(g.renderer);
	SDL_DestroyWindow(g.window);
	SDL_CloseAudio();
	SDL_Quit();
	return 0;
}

