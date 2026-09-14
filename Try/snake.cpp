#include <algorithm>
#include <chrono>
#include <csignal>
#include <deque>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

// Linux/macOS terminal game. Compile: g++ -std=c++17 snake.cpp -o snake
namespace {
constexpr int width = 30;
constexpr int height = 16;
volatile std::sig_atomic_t interrupted = 0;

void onSignal(int) { interrupted = 1; }

// Restore the terminal automatically on normal exit and exceptions.
class Terminal {
    termios original{};
public:
    Terminal() {
        if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO))
            throw std::runtime_error("Please run this game in an interactive terminal.");
        if (tcgetattr(STDIN_FILENO, &original) != 0)
            throw std::runtime_error("Cannot read terminal settings.");
        termios settings = original;
        settings.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &settings) != 0)
            throw std::runtime_error("Cannot configure terminal.");
        std::cout << "\033[?1049h\033[?25l" << std::flush;
    }
    ~Terminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &original);
        std::cout << "\033[?25h\033[?1049l" << std::flush;
    }
    Terminal(const Terminal&) = delete;
    Terminal& operator=(const Terminal&) = delete;
};

struct Point {
    int x, y;
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

int readKey() {
    fd_set input;
    FD_ZERO(&input);
    FD_SET(STDIN_FILENO, &input);
    timeval timeout{0, 0};
    if (select(STDIN_FILENO + 1, &input, nullptr, nullptr, &timeout) <= 0)
        return -1;
    unsigned char key;
    return read(STDIN_FILENO, &key, 1) == 1 ? key : -1;
}

Point newFood(const std::deque<Point>& snake, std::mt19937& rng) {
    std::vector<Point> empty;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            if (std::find(snake.begin(), snake.end(), Point{x, y}) == snake.end())
                empty.push_back({x, y});
    std::uniform_int_distribution<std::size_t> pick(0, empty.size() - 1);
    return empty[pick(rng)];
}

void draw(const std::deque<Point>& snake, Point food, int score) {
    std::vector<std::string> board(height, std::string(width, ' '));
    board[food.y][food.x] = '*';
    for (const auto& p : snake) board[p.y][p.x] = 'o';
    board[snake.front().y][snake.front().x] = '@';
    std::string frame = "\033[HSNAKE | Score: " + std::to_string(score) + "\033[K\n";
    frame += "W/A/S/D: move | Q: quit\033[K\n";
    frame += '+' + std::string(width, '-') + "+\n";
    for (const auto& row : board) frame += '|' + row + "|\n";
    frame += '+' + std::string(width, '-') + "+\n";
    frame += "@: head  o: body  *: food\033[K\n";
    std::cout << frame << std::flush;
}
} // namespace

int main() {
    int score = 0;
    std::string result = "Thanks for playing!";
    try {
        Terminal terminal;
        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);
        std::mt19937 rng(std::random_device{}());
        std::deque<Point> snake{{width / 2, height / 2},
                                {width / 2 - 1, height / 2},
                                {width / 2 - 2, height / 2}};
        Point direction{1, 0};
        Point food = newFood(snake, rng);
        draw(snake, food, score);
        bool quit = false;
        while (!quit && !interrupted) {
            const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(std::max(70, 180 - score * 2));
            bool turned = false;
            while (std::chrono::steady_clock::now() < deadline && !interrupted) {
                int key = readKey();
                if (key == 'q' || key == 'Q') { quit = true; break; }
                Point next = direction;
                bool movement = true;
                switch (key) {
                    case 'w': case 'W': next = {0, -1}; break;
                    case 's': case 'S': next = {0, 1}; break;
                    case 'a': case 'A': next = {-1, 0}; break;
                    case 'd': case 'D': next = {1, 0}; break;
                    default: movement = false;
                }
                // Only one turn per tick; reversing into the body is forbidden.
                if (movement && !turned &&
                    !(next.x == -direction.x && next.y == -direction.y) &&
                    !(next == direction)) {
                    direction = next;
                    turned = true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            if (quit || interrupted) break;
            Point head{snake.front().x + direction.x, snake.front().y + direction.y};
            bool eating = head == food;
            // The tail moves away on a tick where no food is eaten.
            auto bodyEnd = eating ? snake.end() : std::prev(snake.end());
            if (head.x < 0 || head.x >= width || head.y < 0 || head.y >= height ||
                std::find(snake.begin(), bodyEnd, head) != bodyEnd) {
                result = "Game over! You hit a wall or your body.";
                break;
            }
            snake.push_front(head);
            if (eating) {
                score += 10;
                if (snake.size() == static_cast<std::size_t>(width * height)) {
                    result = "You win! You filled the board.";
                    break;
                }
                food = newFood(snake, rng);
            } else {
                snake.pop_back();
            }
            draw(snake, food, score);
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << result << " Final score: " << score << '\n';
    return 0;
}
