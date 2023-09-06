import numpy as np


def get_vector_direction(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def get_vector_magnitude(vector: np.ndarray) -> float:
    return np.linalg.norm(vector)


def main():
    print(get_vector_magnitude(np.array([0.0, 0.0, 0.0])))


if __name__ == '__main__':
    main()
