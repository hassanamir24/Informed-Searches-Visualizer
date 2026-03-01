# Informed-Searches-Visualizer


### Requirements
- Python 3.8+
- pygame

### Installation
```bash
pip install pygame
```

### Running
```bash
python main.py
```

### Features
- **Algorithms:** A* Search, Greedy Best-First Search
- **Heuristics:** Manhattan Distance, Euclidean Distance
- **Dynamic Mode:** Random obstacles spawn while agent navigates; agent re-plans automatically
- **Interactive Editor:** Left-click to add walls, right-click to remove
- **Metrics Dashboard:** Nodes expanded, path cost, execution time

### Controls
| Action | Control |
|--------|---------|
| Add wall | Left-click grid cell |
| Remove wall | Right-click grid cell |
| Select algorithm | Buttons in panel |
| Select heuristic | Buttons in panel |
| Run search | ▶ Run Search button |
| Toggle dynamic mode | Dynamic Mode button |
| Reset grid | ↺ Reset Grid button |

### Color Legend
| Color | Meaning |
|-------|---------|
| Green | Start node |
| Red | Goal node |
| Orange | Agent (current position) |
| Teal | Final path |
| Blue | Expanded/visited nodes |
| Yellow | Frontier nodes |
| Black | Walls/obstacles |
