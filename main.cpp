#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <tuple>

#include <SDL2/SDL.h>

int const WIDTH = 800;
int const HEIGHT = 600;

class Object; // declaration

struct global_t {
	SDL_Window * window = NULL;
	SDL_Renderer * renderer = NULL;

	// Objects
	std::vector<Object*> sceneObjects;

	// random
	std::random_device rd;
	std::default_random_engine eng;
	std::uniform_real_distribution<float> rand;

};

global_t w;


class Object {
	protected:
		float x;
		float y;
		int scale;
		std::tuple<Uint8, Uint8, Uint8, Uint8> color;
	public:
		Object (float x, float y, int scale, std::tuple<Uint8, Uint8, Uint8, Uint8> color) 
			: x(x), y(y), scale(scale), color(color) {}
		virtual int draw() = 0;
		virtual int update() = 0;
};

class Boids : public Object {
	private:
		float direction; // [0,2*pi[
		// x,y coordinates that are inherited represent the middle of the base line
	public:
		Boids (float x, float y, int scale, std::tuple<Uint8, Uint8, Uint8, Uint8> color, float direction) 
		: Object(x,y,scale,color) {
			this->direction = direction;
		}

		int draw() override {
			// Draws the boid to the screen

			Uint8 r, g, b, a;
			std::tie(r,g,b,a) = color;

			float orthogonal = direction + M_PI / 2;
			if (orthogonal > 2*M_PI) orthogonal -= 2*M_PI;

			float delta_x = scale / 2 * std::sinf(orthogonal); // l * sin(PI - theta')
			float delta_y = - scale / 2 * std::cosf(orthogonal); // l * cos(PI - theta')
			float delta_x_normal = 1.5*scale*std::sinf(direction);
			float delta_y_normal = -1.5*scale*std::cosf(direction);

			const std::vector< SDL_Vertex > verts =
				{
					SDL_Vertex {SDL_FPoint{x-delta_x, y-delta_y}, SDL_Color{r,g,b,a}, SDL_FPoint{ 0 }},
					SDL_Vertex {SDL_FPoint{x+delta_x, y+delta_y}, SDL_Color{r,g,b,a}, SDL_FPoint{ 0 }},
					SDL_Vertex {SDL_FPoint{x+delta_x_normal, y+delta_y_normal}, SDL_Color{r,g,b,a}, SDL_FPoint{ 0 }}
				};
			
			return SDL_RenderGeometry(w.renderer, nullptr, verts.data(), verts.size(), nullptr, 0);
		}

		int update() override {
			// Updates the boid
			return 0;
		}
};

void do_render() {
	// Reset to black screen
	SDL_SetRenderDrawColor(w.renderer, 0u, 0u, 0u, SDL_ALPHA_OPAQUE);
	SDL_RenderClear(w.renderer);

	for (Object* x : w.sceneObjects){
		x->draw();
	}

	SDL_RenderPresent(w.renderer);
}

void do_update() {
	// Update the scene
	for (Object* x : w.sceneObjects){
		x->update();
	}
}

void init_scene(){
	auto color = std::make_tuple<Uint8, Uint8, Uint8, Uint8>(255,0,0,255);
	Boids * b1 =  new Boids(WIDTH/2, HEIGHT/2, 25, color, 0.5);
	w.sceneObjects.push_back(b1);
}

int main(int argc, char ** argv)
{
	std::srand(time(NULL));

	int status;

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_EVENTS | SDL_INIT_TIMER) != 0) {
		return 1;
	}

	w.window = SDL_CreateWindow("BOIDS",
			SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
			WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
	if (not w.window) {
		return 1;
	}

	// get the default renderer
	w.renderer = SDL_CreateRenderer(w.window, -1, 0);
	if (not w.renderer) {
		return 1;
	}

	init_scene();
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

	SDL_DestroyRenderer(w.renderer);
	SDL_DestroyWindow(w.window);
	SDL_CloseAudio();
	SDL_Quit();
	return 0;
}
