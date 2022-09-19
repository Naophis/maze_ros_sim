#include "path_creator.hpp"

PathCreator::PathCreator(/* args */) {}

PathCreator::~PathCreator() {}

int PathCreator::get_dist_val(int x, int y) {
  //
  return lgc->get_dist_val(x, y);
}

void PathCreator::updateVectorMap(const bool isSearch) {
  lgc->updateVectorMap(isSearch);
}

void PathCreator::path_reflash() {
  path_s.clear();
  path_t.clear();
}

void PathCreator::append_path_s(float val) {
  path_s.emplace_back(val); //
}
void PathCreator::append_path_t(int val) {
  path_t.emplace_back(val); //
}
void PathCreator::add_path_s(int idx, float val) {
  path_s[idx] += val; //
}

void PathCreator::setNextRootDirectionPath(int x, int y, Direction now_dir,
                                           Direction dir, float &val,
                                           Direction &next_dir) {
  const bool isWall = lgc->existWall(x, y, dir);
  const bool step = lgc->isStep(x, y, dir);
  const float dist =
      isWall ? vector_max_step_val : lgc->getDistVector(x, y, dir);

  if (static_cast<int>(now_dir) * static_cast<int>(dir) == 8)
    return;
  if (!isWall && dist < val) {
    next_dir = dir;
    val = dist;
  }
}
Motion PathCreator::get_next_motion(Direction now_dir,
                                    Direction next_direction) {
  if (now_dir == next_direction)
    return Motion::Straight;

  if (now_dir == Direction::North) {
    if (next_direction == Direction::East)
      return Motion::TurnRight;
    else if (next_direction == Direction::West)
      return Motion::TurnLeft;
  } else if (now_dir == Direction::East) {
    if (next_direction == Direction::South)
      return Motion::TurnRight;
    else if (next_direction == Direction::North)
      return Motion::TurnLeft;
  } else if (now_dir == Direction::West) {
    if (next_direction == Direction::North)
      return Motion::TurnRight;
    else if (next_direction == Direction::South)
      return Motion::TurnLeft;
  } else if (now_dir == Direction::South) {
    if (next_direction == Direction::West)
      return Motion::TurnRight;
    else if (next_direction == Direction::East)
      return Motion::TurnLeft;
  }
  return Motion::Back;
}
Direction PathCreator::get_next_pos(int &x, int &y, Direction dir,
                                    Direction next_direction) {
  if (next_direction == Direction::Undefined)
    if (dir == Direction::North)
      next_direction = Direction::South;
    else if (dir == Direction::East)
      next_direction = Direction::West;
    else if (dir == Direction::West)
      next_direction = Direction::East;
    else if (dir == Direction::South)
      next_direction = Direction::North;

  if (next_direction == Direction::North)
    y++;
  else if (next_direction == Direction::East)
    x++;
  else if (next_direction == Direction::West)
    x--;
  else if (next_direction == Direction::South)
    y--;

  return next_direction;
}

void PathCreator::addCheckQ(int x, int y) {
  for (int i = 0; i < checkQlength; i++) {
    if ((checkQ[i] & 0xf0) >> 4 == x && (checkQ[i] & 0x0f) == y) {
      return;
    }
  }
  for (int i = 0; i < checkQlength; i++) {
    if (checkQ[i] == 0) {
      checkQ[i] = x * 16 + y;
      return;
    }
  }
}
void PathCreator::checkOtherRoot(int x, int y, float now, Direction now_dir) {
  // int temp = 0;
  // float a[4];
  // int index = 0;
  // for (const auto d : direction_list) {
  //   float dist = lgc->getDistVector(x, y, d);
  //   if ((static_cast<int>(now_dir) * static_cast<int>(d)) != 8 &&
  //       !lgc->existWall(x, y, d) && dist < now) {
  //     temp += d;
  //     a[index] = dist;
  //     index++;
  //   }
  // }
  // float ch = a[0];
  // for (int i = 0; i < 4; i++)
  //   if (a[i] == 0)
  //     continue;
  //   else if (a[i] != ch)
  //     ch = a[i];

  // if (temp == 0)
  //   return;

  // if ((temp == 0x01) || (temp == 0x02) || (temp == 0x04) || (temp == 0x08)) {
  // } else {
  //   checkMap[x][y] = temp;
  //   addCheckQ(x, y);
  // }
}

void PathCreator::priorityStraight2(int x, int y, Direction now_dir,
                                    Direction dir, float &dist_val,
                                    Direction &next_dir) {
  const bool isWall = lgc->existWall(x, y, dir);
  const bool step = lgc->isStep(x, y, dir);
  const float dist = isWall ? MAX : lgc->getDistVector(x, y, dir);
  if (static_cast<int>(now_dir) * static_cast<int>(dir) == 8)
    return;
  if (!isWall && step && dist <= dist_val) {
    next_dir = dir;
    dist_val = dist;
  }
}
void PathCreator::clearCheckMap() {
  // for (char i = 0; i < 16; i++)
  //   for (char j = 0; j < 16; j++)
  //     checkMap[i][j] = 0;
}
void PathCreator::path_create(bool is_search) {
  Direction next_dir = Direction::North;
  Direction now_dir = next_dir;
  unsigned int idx = 0;
  Direction dirLog[3];
  bool b1 = false;
  bool b2 = false;

  int x = 0;
  int y = 1;

  path_reflash();
  clearCheckMap();
  lgc->updateVectorMap(is_search);

  path_s.emplace_back(3);
  float dist_val;
  while (true) {
    now_dir = next_dir;
    dirLog[2] = dirLog[1];
    dirLog[1] = dirLog[0];
    dirLog[0] = now_dir;

    dist_val = MAX;
    next_dir = Direction::Undefined;

    if (lgc->arrival_goal_position(x, y)) {
      add_path_s(idx, 3);
      path_t.emplace_back(255);
      // path_t.emplace_back(0);
      path_size = idx;
      return;
    }
    const float position = lgc->getDistVector(x, y, now_dir);

    setNextRootDirectionPath(x, y, now_dir, Direction::North, dist_val,
                             next_dir);
    setNextRootDirectionPath(x, y, now_dir, Direction::East, dist_val,
                             next_dir);
    setNextRootDirectionPath(x, y, now_dir, Direction::West, dist_val,
                             next_dir);
    setNextRootDirectionPath(x, y, now_dir, Direction::South, dist_val,
                             next_dir);
    if (dirLog[0] == dirLog[1] || dirLog[0] != dirLog[2])
      priorityStraight2(x, y, now_dir, dirLog[0], dist_val, next_dir);
    else
      priorityStraight2(x, y, now_dir, dirLog[1], dist_val, next_dir);

    Motion nextMotion = get_next_motion(now_dir, next_dir);

    if (nextMotion == Motion::Straight) {
      add_path_s(idx, 2);
    } else if (nextMotion == Motion::TurnRight) {
      path_t.emplace_back(static_cast<int>(TurnDirection::Right));
      path_s.emplace_back(2);
      idx++;
    } else if (nextMotion == Motion::TurnLeft) {
      path_t.emplace_back(static_cast<int>(TurnDirection::Left));
      path_s.emplace_back(2);
      idx++;
    } else {
      path_t.emplace_back(0);
      break;
    }
    next_dir = get_next_pos(x, y, now_dir, next_dir);
  }
  path_size = idx;
}

void PathCreator::convert_large_path(bool b1) {
  int i = 0;
  int finish = 0;
  for (i = 0; i < path_size; i++) {
    if (path_s[i + 1] == 2) {
      if ((path_s[i] > 2) && path_s[i + 2] > 2) {
        if (path_t[i] == path_t[i + 1]) {
          if (path_t[i] == 1) {
            path_t[i] = 3;
            path_t[i + 1] = 254;
          } else if (path_t[i] == 2) {
            path_t[i] = 4;
            path_t[i + 1] = 254;
          }
          path_s[i] -= 1;
          path_s[i + 2] -= 1;
          i++;
        }
      }
    }
    if (path_t[i] == 0) {
      break;
    }
  }
  for (i = 0; i < path_size; i++) {
    if ((path_s[i] > 2) && (path_s[i + 1] > 2)) {
      if (path_t[i] == 1) {
        path_t[i] = 5;
        path_s[i + 1] -= 1;
        path_s[i] -= 1;
      } else if (path_t[i] == 2) {
        path_t[i] = 6;
        path_s[i + 1] -= 1;
        path_s[i] -= 1;
      }
    }
    if (path_t[i] == 0) {
      break;
    }
  }
  if (b1) {
    if (path_size >= 2 && path_s[0] == 2 && path_t[0] == 1 && path_s[1] > 2) {
      path_s[0] -= 1;
      path_s[1] -= 1;
      path_t[0] = 5;
    }
    i = 0;
    while (path_t[i] != 0) {
      i++;
    }
    finish = i;
    if (finish >= 1) {
      if (path_s[finish] == 2 && path_s[finish - 1] > 2 &&
          (path_t[finish - 1] == 1 || path_t[finish - 1] == 2)) {
        path_s[finish - 1] -= 1;
        path_s[finish] -= 1;
        if (path_t[finish - 1] == 1) {
          path_t[finish - 1] = 5;
        } else if (path_t[finish - 1] == 2) {
          path_t[finish - 1] = 6;
        }
      }
    }
    if (finish > 2) {
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] > 2 && path_t[finish - 1] == 1 &&
          path_t[finish - 2] == 1) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = 3;
        path_t[finish - 1] = 254;
      }
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] > 2 && path_t[finish - 1] == 2 &&
          path_t[finish - 2] == 2) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = 4;
        path_t[finish - 1] = 254;
      }
    }
    if (finish == 2) {
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] >= 2 && path_t[finish - 1] == 1 &&
          path_t[finish - 2] == 1) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = 3;
        path_t[finish - 1] = 254;
      }
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] >= 2 && path_t[finish - 1] == 2 &&
          path_t[finish - 2] == 2) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = 4;
        path_t[finish - 1] = 254;
      }
    }
  }
}

void PathCreator::diagonalPath(bool isFull, bool a1) {
  int i = 0;
  int j = 0;
  int dir = 0;
  int check = 0;
  bool diaMode = false;
  int m = 0;
  bool check2 = false;
  bool check3 = false;

  bool _virtual = !a1;
  bool flag = false;
  while (path_t[i] != 0) {
    check = 0;
    if (_virtual) {
      a1 = false;
      if (path_s[i] > 2) {
        flag = true;
      }
      if (flag /* && i != 0*/) {
        for (m = i + 1;; m++) {
          if (path_s[m] > 2) {
            a1 = true;
            break;
          }
          if (path_t[m] == 0) {
            a1 = false;
            break;
          }
        }
      }
    }

    //		if (/*i > 0 &&*/m != 0 && path_s[m] > 2) {
    //			a1 = true;
    //		}
    if (path_t[i] == R && (a1 ? true : path_s[i] > 2)) {
      dir = R;
      for (j = i + 1; path_t[j] != dir; j++) {
        if (path_t[j] == R && path_s[j] == 2) {
          dir = R;
        } else if (path_t[j] == L && path_s[j] == 2) {
          dir = L;
        } else {
          break;
        }
        check++;
      }
    } else if (path_t[i] == L && (a1 ? true : path_s[i] > 2)) {
      dir = L;
      for (j = i + 1; path_t[j] != dir; j++) {
        if (path_t[j] == R && path_s[j] == 2) {
          dir = R;
        } else if (path_t[j] == L && path_s[j] == 2) {
          dir = L;
        } else {
          break;
        }
        check++;
      }
    }
    if (check != 0) {
      j -= 1;
      if ((i != 0 && path_s[i] == 2 && path_t[i] == path_t[i - 1] &&
           path_s[i - 1] > 2)) {
        //      ||    (i == 1 && path_s[i] == 2 && path_t[i] == path_t[i - 1] &&
        //      path_s[i - 1] > 0)) {
        if (path_t[i] == R) {
          path_t[i - 1] = 9;
          check3 = true;
        } else if (path_t[i] == L) {
          path_t[i - 1] = 10;
          check3 = true;
        }
        if (j != 0 && path_s[j + 2] > 2 && path_t[j] == path_t[j + 1]) {
          if (path_t[j] == R) {
            path_t[j + 1] = 9;
            check2 = true;
          } else if (path_t[j] == L) {
            path_t[j + 1] = 10;
            check2 = true;
          }
          path_s[j + 1] = check + 1;
        } else {
          if (path_t[j] == R) {
            path_t[j] = 7;
          } else if (path_t[j] == L) {
            path_t[j] = 8;
          }
          path_s[j] = check + 1;
        }
      } else {
        int memory = 0;
        if (path_t[i] == R) {
          path_t[i] = 7;
          memory = R;
        } else if (path_t[i] == L) {
          path_t[i] = 8;
          memory = L;
        }
        if (j != 0 && path_s[j + 2] > 2 && path_t[j] == path_t[j + 1]) {
          if (path_t[j] == R) {
            path_t[j + 1] = 9;
            check2 = true;
          } else if (path_t[j] == L) {
            path_t[j + 1] = 10;
            check2 = true;
          }
          path_s[j + 1] = check + 1;
        } else {
          if (!a1) {
            if (path_s[j + 1] > 2) {
              if (path_t[j] == R) {
                path_t[j] = 7;
              } else if (path_t[j] == L) {
                path_t[j] = 8;
              }
              path_s[j] = check + 1;
            } else {
              path_t[i] = memory;
              check3 = false;
              check2 = false;
              i = j;
              i++;
              continue;
            }
          } else {
            if (path_t[j] == R) {
              path_t[j] = 7;
            } else if (path_t[j] == L) {
              path_t[j] = 8;
            }
            path_s[j] = check + 1;
          }
        }
      }
      if (check3) {
        for (int k = i; k < j; k++) {
          path_t[k] = 254;
        }
      } else {
        for (int k = i + 1; k < j; k++) {
          path_t[k] = 254;
        }
      }
      if (check2) {
        path_t[j] = 254;
      }
      check3 = false;
      check2 = false;
      i = j;
    }
    i++;
  }
  i = 0;

  while (path_t[i] != 0) {
    if (path_t[i] == 7 && path_t[i + 1] == 7 && path_s[i + 1] == 2) {
      path_t[i] = 11;
      path_t[i + 1] = 254;
    }
    if (path_t[i] == 8 && path_t[i + 1] == 8 && path_s[i + 1] == 2) {
      path_t[i] = 12;
      path_t[i + 1] = 254;
    }
    i++;
  }
  i = 0;
  diaMode = false;
  while (path_t[i] != 0) {
    if (!diaMode) {
      if (path_t[i] == 7 || path_t[i] == 8 || path_t[i] == 9 ||
          path_t[i] == 10) {
        path_s[i] -= 1;
        diaMode = true;
      }
    } else if (diaMode) {
      if (path_t[i] == 7 || path_t[i] == 8 || path_t[i] == 9 ||
          path_t[i] == 10) {
        path_s[i + 1] -= 1;
        diaMode = false;
      }
    }
    i++;
  }
  pathOffset();
  // TODO APEC対応でコメントアウト中
  //	if (isFull) {
  //		i = 0;
  //		for (i = 0; i < 255; i++) {
  //			if (i > 0 && path_t[i] == 255 && path_s[i] == 1) {
  //				if (path_t[i - 1] == 7 || path_t[i - 1] == 8) {
  //					path_t[i - 1] = 255;
  //					path_s[i - 1] += 1;
  //				}
  //			}
  //			if (path_t[i] == 255) {
  //				break;
  //			}
  //		}
  //	}
}

void PathCreator::pathOffset() {
  int i = 0;
  for (i = 0; i < path_size; i++) {
    if (path_t[i] == 0) {
      path_t[i] = 0xff;
      break;
    }
  }
  i = 0;
  while (i < path_size && path_t[i] != 0xff) {
    if (i > 0) {
      if (path_t[i] == 254) {
        path_s[i] = 0;
        path_t[i] = 0;
      }
    }
    i++;
  }
  i = 0;
  while (i < path_size && path_t[i] != 0xff) {
    while (path_t[i] == 0) {
      for (int j = i; path_t[j] != 0xff; j++) {
        path_s[j] = path_s[j + 1];
        path_t[j] = path_t[j + 1];
      }
    }
    i++;
  }
}

// float PathCreator::drawChangePathRoot(char goalX, char goalY, char isFull) {
//   float pathVariation[4];
//   unsigned char check = 0;
//   unsigned char x = 0;
//   unsigned char y = 0;
//   unsigned char checkPoint[4];
//   unsigned char temp = 255;
//   unsigned char i = 0;
//   unsigned char j = 0;
//   unsigned char k = 0;
//   unsigned char turnVariety[4][2];
//   unsigned char Priority = 1;
//   unsigned char p = 0;
//   unsigned char q = 0;
//   unsigned char checkker = 0;
//   unsigned int turnLisk[4];
//   int tempLisk = 0;

//   // deadEnd(goalX, goalY);
//   //	updateVectorMap(goalX, goalY, false, isFull);
//   vectorDistUpdate(goalX, goalY, isFull);
//   for (i = 0; checkQ[i] != 0; i++) {
//     pathVariation[0] = 255;
//     pathVariation[1] = 255;
//     pathVariation[2] = 255;
//     pathVariation[3] = 255;
//     checkPoint[0] = 0;
//     checkPoint[1] = 0;
//     checkPoint[2] = 0;
//     checkPoint[3] = 0;
//     turnLisk[0] = 255;
//     turnLisk[1] = 255;
//     turnLisk[2] = 255;
//     turnLisk[3] = 255;
//     x = (checkQ[i] / 16);
//     y = checkQ[i] & 0x0f;
//     if (checkMap[x][y] != 0 && (checkMap[x][y] & 0xf0) == 0) {
//       if ((checkMap[x][y] & 0x01) == 0x01) {
//         pathVariation[0] = pathCreateChange(goalX, goalY, x, y, North);
//         // myprintf("north(%d,%d) %f\r\n", x, y, pathVariation[0]);
//         if (checkTurningPoint) {
//           checkPoint[0] = North;
//         }
//         if (pathVariation[0] != 255) {
//           turnLisk[0] = countTurnLisk();
//           for (k = 0; path_t[k] != 255; k++) {
//             if (path_t[k] < 7) {
//               turnVariety[0][0] += 1;
//             } else {
//               turnVariety[0][1] += 1;
//             }
//           }
//         }
//       }
//       if ((checkMap[x][y] & 0x02) == 0x02) {
//         pathVariation[1] = pathCreateChange(goalX, goalY, x, y, East);
//         // myprintf("east(%d,%d) %f\r\n", x, y, pathVariation[1]);
//         if (checkTurningPoint) {
//           checkPoint[0] = East;
//         }
//         if (pathVariation[1] != 255) {
//           turnLisk[1] = countTurnLisk();
//           for (k = 0; path_t[k] != 255; k++) {
//             if (path_t[k] < 7) {
//               turnVariety[1][0] += 1;
//             } else {
//               turnVariety[1][1] += 1;
//             }
//           }
//         }
//       }
//       if ((checkMap[x][y] & 0x04) == 0x04) {
//         pathVariation[2] = pathCreateChange(goalX, goalY, x, y, West);
//         // myprintf("west(%d,%d) %f\r\n", x, y, pathVariation[2]);
//         if (checkTurningPoint) {
//           checkPoint[0] = West;
//         }
//         if (pathVariation[2] != 255) {
//           turnLisk[2] = countTurnLisk();
//           for (k = 0; path_t[k] != 255; k++) {
//             if (path_t[k] < 7) {
//               turnVariety[2][0] += 1;
//             } else {
//               turnVariety[2][1] += 1;
//             }
//           }
//         }
//       }
//       if ((checkMap[x][y] & 0x08) == 0x08) {
//         pathVariation[3] = pathCreateChange(goalX, goalY, x, y, South);
//         // myprintf("south(%d,%d) %f\r\n", x, y, pathVariation[3]);
//         if (checkTurningPoint) {
//           checkPoint[0] = South;
//         }
//         if (pathVariation[3] != 255) {
//           turnLisk[3] = countTurnLisk();
//           for (k = 0; path_t[k] != 255; k++) {
//             if (path_t[k] < 7) {
//               turnVariety[3][0] += 1;
//             } else {
//               turnVariety[3][1] += 1;
//             }
//           }
//         }
//       }
//       temp = 255;
//       tempLisk = 0;
//       //			myprintf("%f	%f	%f	%f\r\n",
//       //pathVariation[0], pathVariation[1], 					pathVariation[2], pathVariation[3]);
//       for (j = 0; j < 4; j++) {
//         if (pathVariation[j] < temp) {
//           temp = pathVariation[j];
//           if (j == 0) {
//             check = 1;
//           } else if (j == 1) {
//             check = 2;
//           } else if (j == 2) {
//             check = 4;
//           } else if (j == 3) {
//             check = 8;
//           }
//         } else if (pathVariation[j] != 255 && pathVariation[j] == temp) {
//           if (tempLisk < turnLisk[j]) {
//             temp = pathVariation[j];
//             tempLisk = turnLisk[j];
//             if (j == 0) {
//               check = 1;
//             } else if (j == 1) {
//               check = 2;
//             } else if (j == 2) {
//               check = 4;
//             } else if (j == 3) {
//               check = 8;
//             }
//           }
//         }
//       }
//       //			myprintf("check=%d", check);
//       if (Priority == 1) {
//         if (pathVariation[0] == pathVariation[1] && pathVariation[0] != 255) {
//           if (turnVariety[0][1] > turnVariety[1][1]) {
//             check = 1;
//           } else if (turnVariety[0][1] < turnVariety[1][1]) {
//             check = 2;
//           }
//         } else if (pathVariation[1] == pathVariation[2] &&
//                    pathVariation[1] != 255) {
//           if (turnVariety[1][1] > turnVariety[2][1]) {
//             check = 2;
//           } else if (turnVariety[1][1] < turnVariety[2][1]) {
//             check = 4;
//           }
//         } else if (pathVariation[2] == pathVariation[3] &&
//                    pathVariation[2] != 255) {
//           if (turnVariety[2][1] > turnVariety[3][1]) {
//             check = 4;
//           } else if (turnVariety[2][1] < turnVariety[3][1]) {
//             check = 8;
//           }
//         } else if (pathVariation[3] == pathVariation[0] &&
//                    pathVariation[3] != 255) {
//           if (turnVariety[0][1] > turnVariety[3][1]) {
//             check = 1;
//           } else if (turnVariety[0][1] < turnVariety[3][1]) {
//             check = 8;
//           }
//         } else if (pathVariation[3] == pathVariation[1] &&
//                    pathVariation[1] != 255) {
//           if (turnVariety[1][1] > turnVariety[3][1]) {
//             check = 2;
//           } else if (turnVariety[1][1] < turnVariety[3][1]) {
//             check = 8;
//           }
//         } else if (pathVariation[0] == pathVariation[2] &&
//                    pathVariation[2] != 255) {
//           if (turnVariety[0][1] > turnVariety[2][1]) {
//             check = 1;
//           } else if (turnVariety[0][1] < turnVariety[2][1]) {
//             check = 4;
//           }
//         }
//       } else {
//         if (pathVariation[0] == pathVariation[1] && pathVariation[0] != 255) {
//           if (turnVariety[0][1] < turnVariety[1][1]) {
//             check = 1;
//           } else if (turnVariety[0][1] > turnVariety[1][1]) {
//             check = 2;
//           }
//         } else if (pathVariation[1] == pathVariation[2] &&
//                    pathVariation[1] != 255) {
//           if (turnVariety[1][1] < turnVariety[2][1]) {
//             check = 2;
//           } else if (turnVariety[1][1] > turnVariety[2][1]) {
//             check = 4;
//           }
//         } else if (pathVariation[2] == pathVariation[3] &&
//                    pathVariation[2] != 255) {
//           if (turnVariety[2][1] < turnVariety[3][1]) {
//             check = 4;
//           } else if (turnVariety[2][1] > turnVariety[3][1]) {
//             check = 8;
//           }
//         } else if (pathVariation[3] == pathVariation[0] &&
//                    pathVariation[3] != 255) {
//           if (turnVariety[0][1] < turnVariety[3][1]) {
//             check = 1;
//           } else if (turnVariety[0][1] > turnVariety[3][1]) {
//             check = 8;
//           }
//         } else if (pathVariation[3] == pathVariation[1] &&
//                    pathVariation[1] != 255) {
//           if (turnVariety[1][1] < turnVariety[3][1]) {
//             check = 2;
//           } else if (turnVariety[1][1] > turnVariety[3][1]) {
//             check = 8;
//           }
//         } else if (pathVariation[0] == pathVariation[2] &&
//                    pathVariation[2] != 255) {
//           if (turnVariety[0][1] < turnVariety[2][1]) {
//             check = 1;
//           } else if (turnVariety[0][1] > turnVariety[2][1]) {
//             check = 4;
//           }
//         }
//       }
//       //			myprintf("check=%d", check);
//       if ((checkPoint[0] == 0 && checkPoint[1] == 0 && checkPoint[2] == 0 &&
//            checkPoint[3] == 0) ||
//           check == 0) {
//         deleteQ(x, y);
//         if (!checkDualCheck(x, y)) {
//           addQ(x, y);
//         }
//         i -= 1;
//       } else {
//         checkker = (check << 4) | (checkMap[x][y] & 0x0f);
//         checkMap[x][y] = checkker;
//       }
//       for (p = 0; p < 4; p++) {
//         for (q = 0; q < 2; q++) {
//           turnVariety[p][q] = 0;
//         }
//       }
//     }
//   }
//   for (i = 0; checkQ[i] != 0; i++) {
//     pathVariation[0] = 255;
//     pathVariation[1] = 255;
//     pathVariation[2] = 255;
//     pathVariation[3] = 255;
//     checkPoint[0] = 0;
//     checkPoint[1] = 0;
//     checkPoint[2] = 0;
//     checkPoint[3] = 0;
//     x = (checkQ[i] / 16);
//     y = checkQ[i] & 0x0f;
//     if ((checkMap[x][y] & 0x01) == 0x01) {
//       pathVariation[0] = pathCreateChange(goalX, goalY, x, y, North);
//       if (checkPoint) {
//         checkPoint[0] = North;
//       }
//       if (pathVariation[0] != 255) {
//         for (k = 0; path_t[k] != 255; k++) {
//           if (path_t[k] < 7) {
//             turnVariety[0][0] += 1;
//           } else {
//             turnVariety[0][1] += 1;
//           }
//         }
//       }
//     }
//     if ((checkMap[x][y] & 0x02) == 0x02) {
//       pathVariation[1] = pathCreateChange(goalX, goalY, x, y, East);
//       if (checkPoint) {
//         checkPoint[0] = East;
//       }
//       if (pathVariation[1] != 255) {
//         for (k = 0; path_t[k] != 255; k++) {
//           if (path_t[k] < 7) {
//             turnVariety[1][0] += 1;
//           } else {
//             turnVariety[1][1] += 1;
//           }
//         }
//       }
//     }
//     if ((checkMap[x][y] & 0x04) == 0x04) {
//       pathVariation[2] = pathCreateChange(goalX, goalY, x, y, West);
//       if (checkPoint) {
//         checkPoint[0] = West;
//       }
//       if (pathVariation[2] != 255) {
//         for (k = 0; path_t[k] != 255; k++) {
//           if (path_t[k] < 7) {
//             turnVariety[2][0] += 1;
//           } else {
//             turnVariety[2][1] += 1;
//           }
//         }
//       }
//     }
//     if ((checkMap[x][y] & 0x08) == 0x08) {
//       pathVariation[3] = pathCreateChange(goalX, goalY, x, y, South);
//       if (checkPoint) {
//         checkPoint[0] = South;
//       }
//       if (pathVariation[3] != 255) {
//         for (k = 0; path_t[k] != 255; k++) {
//           if (path_t[k] < 7) {
//             turnVariety[3][0] += 1;
//           } else {
//             turnVariety[3][1] += 1;
//           }
//         }
//       }
//     }
//     //		myprintf("%f	%f	%f	%f\r\n", pathVariation[0],
//     //pathVariation[1], 				pathVariation[2], pathVariation[3]);
//     temp = 255;
//     tempLisk = 0;
//     for (j = 0; j < 4; j++) {
//       if (pathVariation[j] < temp) {
//         temp = pathVariation[j];
//         if (j == 0) {
//           check = 1;
//         } else if (j == 1) {
//           check = 2;
//         } else if (j == 2) {
//           check = 4;
//         } else if (j == 3) {
//           check = 8;
//         }
//         //			} else if (pathVariation[j] != 255 &&
//         //pathVariation[j] == temp) { 				if (tempLisk < turnLisk[j]) { 					temp =
//         //pathVariation[j]; 					tempLisk = turnLisk[j]; 					if (j == 0) { 						check = 1; 					}
//         //else if (j == 1) { 						check = 2; 					} else if (j == 2) { 						check = 4; 					} else
//         //if (j == 3) { 						check = 8;
//         //					}
//         //				}
//       }
//     }
//     //		myprintf("check=%d", check);
//     if (Priority == 1) {
//       if (pathVariation[0] == pathVariation[1] && pathVariation[0] != 255) {
//         if (turnVariety[0][1] > turnVariety[1][1]) {
//           check = 1;
//         } else if (turnVariety[0][1] < turnVariety[1][1]) {
//           check = 2;
//         }
//       } else if (pathVariation[1] == pathVariation[2] &&
//                  pathVariation[1] != 255) {
//         if (turnVariety[1][1] > turnVariety[2][1]) {
//           check = 2;
//         } else if (turnVariety[1][1] < turnVariety[2][1]) {
//           check = 4;
//         }
//       } else if (pathVariation[2] == pathVariation[3] &&
//                  pathVariation[2] != 255) {
//         if (turnVariety[2][1] > turnVariety[3][1]) {
//           check = 4;
//         } else if (turnVariety[2][1] < turnVariety[3][1]) {
//           check = 8;
//         }
//       } else if (pathVariation[3] == pathVariation[0] &&
//                  pathVariation[3] != 255) {
//         if (turnVariety[0][1] > turnVariety[3][1]) {
//           check = 1;
//         } else if (turnVariety[0][1] < turnVariety[3][1]) {
//           check = 8;
//         }
//       } else if (pathVariation[3] == pathVariation[1] &&
//                  pathVariation[1] != 255) {
//         if (turnVariety[1][1] > turnVariety[3][1]) {
//           check = 2;
//         } else if (turnVariety[1][1] < turnVariety[3][1]) {
//           check = 8;
//         }
//       } else if (pathVariation[0] == pathVariation[2] &&
//                  pathVariation[2] != 255) {
//         if (turnVariety[0][1] > turnVariety[2][1]) {
//           check = 1;
//         } else if (turnVariety[0][1] < turnVariety[2][1]) {
//           check = 4;
//         }
//       }
//     } else {
//       if (pathVariation[0] == pathVariation[1] && pathVariation[0] != 255) {
//         if (turnVariety[0][1] < turnVariety[1][1]) {
//           check = 1;
//         } else if (turnVariety[0][1] > turnVariety[1][1]) {
//           check = 2;
//         }
//       } else if (pathVariation[1] == pathVariation[2] &&
//                  pathVariation[1] != 255) {
//         if (turnVariety[1][1] < turnVariety[2][1]) {
//           check = 2;
//         } else if (turnVariety[1][1] > turnVariety[2][1]) {
//           check = 4;
//         }
//       } else if (pathVariation[2] == pathVariation[3] &&
//                  pathVariation[2] != 255) {
//         if (turnVariety[2][1] < turnVariety[3][1]) {
//           check = 4;
//         } else if (turnVariety[2][1] > turnVariety[3][1]) {
//           check = 8;
//         }
//       } else if (pathVariation[3] == pathVariation[0] &&
//                  pathVariation[3] != 255) {
//         if (turnVariety[0][1] < turnVariety[3][1]) {
//           check = 1;
//         } else if (turnVariety[0][1] > turnVariety[3][1]) {
//           check = 8;
//         }
//       } else if (pathVariation[3] == pathVariation[1] &&
//                  pathVariation[1] != 255) {
//         if (turnVariety[1][1] < turnVariety[3][1]) {
//           check = 2;
//         } else if (turnVariety[1][1] > turnVariety[3][1]) {
//           check = 8;
//         }
//       } else if (pathVariation[0] == pathVariation[2] &&
//                  pathVariation[2] != 255) {
//         if (turnVariety[0][1] < turnVariety[2][1]) {
//           check = 1;
//         } else if (turnVariety[0][1] > turnVariety[2][1]) {
//           check = 4;
//         }
//       }
//     }
//     if (checkPoint[0] == 0 && checkPoint[1] == 0 && checkPoint[2] == 0 &&
//         checkPoint[3] == 0) {
//       deleteQ(x, y);
//       if (!checkDualCheck(x, y)) {
//         addQ(x, y);
//       }
//       i -= 1;
//     } else {
//       int checkker = (check << 4) | (checkMap[x][y] & 0x0f);
//       checkMap[x][y] = checkker;
//       //			myprintf("(%2x,%2x)=%2x\r\n", x, y, checkker);
//     }
//     for (p = 0; p < 4; p++) {
//       for (q = 0; q < 2; q++) {
//         turnVariety[p][q] = 0;
//       }
//     }
//   }
//   return pathCreateChange(goalX, goalY, 0, 0, 1);
// }