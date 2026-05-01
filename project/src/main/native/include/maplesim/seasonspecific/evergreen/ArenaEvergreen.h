#include "maplesim/simulation/SimulatedArena.h"

namespace maplesim {

/**
 *
 *
 * <h1>A field with no obstacles </h1>
 *
 * <p>This class represents the playing field for the simulated Evergreen Field
 *
 * <p>It extends {@link SimulatedArena}.
 */
class ArenaEvergreen : SimulatedArena {
   public:
    class EvergreenFieldObstacleMap : public FieldMap {
       public:
        /**
         * Used for adding walls when desired.
         *
         * @return this, for chaining.
         */
        EvergreenFieldObstacleMap& WithWalls();
    };

    /**
     *
     *
     * <h1>A field with no obstacles </h1>
     *
     * <p>This class represents the playing field for the simulated Evergreen Field
     *
     * <p>It extends {@link SimulatedArena}.
     *
     * @param withWalls used to add wall obstacles when desired.
     */
    inline ArenaEvergreen(bool withWalls)
          : SimulatedArena{withWalls ? EvergreenFieldObstacleMap{} : EvergreenFieldObstacleMap{}.WithWalls()} {}
};

}  // namespace maplesim