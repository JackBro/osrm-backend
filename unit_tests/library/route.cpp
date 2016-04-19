#include <boost/test/test_case_template.hpp>
#include <boost/test/unit_test.hpp>

#include "args.hpp"
#include "coordinates.hpp"
#include "equal_json.hpp"
#include "fixture.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"
#include "osrm/json_container.hpp"
#include "osrm/osrm.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/status.hpp"

BOOST_AUTO_TEST_SUITE(route)

BOOST_AUTO_TEST_CASE(test_route_same_coordinates_fixture)
{
    const auto args = get_args();
    auto osrm = getOSRM(args.at(0));

    using namespace osrm;

    RouteParameters params;
    params.steps = true;
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());

    json::Object result;
    const auto rc = osrm.Route(params, result);
    BOOST_CHECK(rc == Status::Ok);

    // unset snapping dependent hint
    for (auto &itr : result.values["waypoints"].get<json::Array>().values)
        itr.get<json::Object>().values["hint"] = "";

    const auto location = json::Array{{{7.437070}, {43.749247}}};

    json::Object reference{
        {{"code", "Ok"},
         {"waypoints",
          json::Array{
              {json::Object{
                   {{"name", "Boulevard du Larvotto"}, {"location", location}, {"hint", ""}}},
               json::Object{
                   {{"name", "Boulevard du Larvotto"}, {"location", location}, {"hint", ""}}}}}},
         {"routes",
          json::Array{{json::Object{
              {{"distance", 0.},
               {"duration", 0.},
               {"geometry", "yw_jGupkl@??"},
               {"legs",
                json::Array{{json::Object{
                    {{"distance", 0.},
                     {"duration", 0.},
                     {"summary", ""},
                     {"steps", json::Array{{json::Object{{{"duration", 0.},
                                                          {"distance", 0.},
                                                          {"geometry", "yw_jGupkl@??"},
                                                          {"name", "Boulevard du Larvotto"},
                                                          {"mode", "driving"},
                                                          {"maneuver", json::Object{{
                                                                           {"type", "depart"},
                                                                           {"location", location},
                                                                           {"bearing_before", 0.},
                                                                           {"bearing_after", 0.},
                                                                       }}}}},

                                            json::Object{{{"duration", 0.},
                                                          {"distance", 0.},
                                                          {"geometry", "yw_jGupkl@"},
                                                          {"name", "Boulevard du Larvotto"},
                                                          {"mode", "driving"},
                                                          {"maneuver", json::Object{{
                                                                           {"type", "arrive"},
                                                                           {"location", location},
                                                                           {"bearing_before", 0.},
                                                                           {"bearing_after", 0.},
                                                                       }}}}}}}}}}}}}}}}}}}};

    CHECK_EQUAL_JSON(reference, result);
}

BOOST_AUTO_TEST_CASE(test_route_same_coordinates)
{
    const auto args = get_args();
    auto osrm = getOSRM(args.at(0));

    using namespace osrm;

    RouteParameters params;
    params.steps = true;
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(get_dummy_location());

    json::Object result;
    const auto rc = osrm.Route(params, result);
    BOOST_CHECK(rc == Status::Ok);

    const auto code = result.values.at("code").get<json::String>().value;
    BOOST_CHECK_EQUAL(code, "Ok");

    const auto &waypoints = result.values.at("waypoints").get<json::Array>().values;
    BOOST_CHECK(waypoints.size() == params.coordinates.size());

    for (const auto &waypoint : waypoints)
    {
        const auto &waypoint_object = waypoint.get<json::Object>();

        // nothing can be said about name, empty or contains name of the street
        const auto name = waypoint_object.values.at("name").get<json::String>().value;
        BOOST_CHECK(((void)name, true));

        const auto location = waypoint_object.values.at("location").get<json::Array>().values;
        const auto longitude = location[0].get<json::Number>().value;
        const auto latitude = location[1].get<json::Number>().value;
        BOOST_CHECK(longitude >= -180. && longitude <= 180.);
        BOOST_CHECK(latitude >= -90. && latitude <= 90.);

        const auto hint = waypoint_object.values.at("hint").get<json::String>().value;
        BOOST_CHECK(!hint.empty());
    }

    const auto &routes = result.values.at("routes").get<json::Array>().values;
    BOOST_REQUIRE_GT(routes.size(), 0);

    for (const auto &route : routes)
    {
        const auto &route_object = route.get<json::Object>();

        const auto distance = route_object.values.at("distance").get<json::Number>().value;
        BOOST_CHECK_EQUAL(distance, 0);

        const auto duration = route_object.values.at("duration").get<json::Number>().value;
        BOOST_CHECK_EQUAL(duration, 0);

        // geometries=polyline by default
        const auto geometry = route_object.values.at("geometry").get<json::String>().value;
        BOOST_CHECK(!geometry.empty());

        const auto &legs = route_object.values.at("legs").get<json::Array>().values;
        BOOST_CHECK(!legs.empty());

        for (const auto &leg : legs)
        {
            const auto &leg_object = leg.get<json::Object>();

            const auto distance = leg_object.values.at("distance").get<json::Number>().value;
            BOOST_CHECK_EQUAL(distance, 0);

            const auto duration = leg_object.values.at("duration").get<json::Number>().value;
            BOOST_CHECK_EQUAL(duration, 0);

            // nothing can be said about summary, empty or contains human readable summary
            const auto summary = leg_object.values.at("summary").get<json::String>().value;
            BOOST_CHECK(((void)summary, true));

            const auto &steps = leg_object.values.at("steps").get<json::Array>().values;
            BOOST_CHECK(!steps.empty());

            for (const auto &step : steps)
            {
                const auto &step_object = step.get<json::Object>();

                const auto distance = step_object.values.at("distance").get<json::Number>().value;
                BOOST_CHECK_EQUAL(distance, 0);

                const auto duration = step_object.values.at("duration").get<json::Number>().value;
                BOOST_CHECK_EQUAL(duration, 0);

                // geometries=polyline by default
                const auto geometry = step_object.values.at("geometry").get<json::String>().value;
                BOOST_CHECK(!geometry.empty());

                // nothing can be said about name, empty or contains way name
                const auto name = step_object.values.at("name").get<json::String>().value;
                BOOST_CHECK(((void)name, true));

                // nothing can be said about mode, contains mode of transportation
                const auto mode = step_object.values.at("mode").get<json::String>().value;
                BOOST_CHECK(!name.empty());

                const auto &maneuver = step_object.values.at("maneuver").get<json::Object>().values;

                const auto location = maneuver.at("location").get<json::Array>().values;
                const auto longitude = location[0].get<json::Number>().value;
                const auto latitude = location[1].get<json::Number>().value;
                BOOST_CHECK(longitude >= -180. && longitude <= 180.);
                BOOST_CHECK(latitude >= -90. && latitude <= 90.);

                const auto bearing_before = maneuver.at("bearing_before").get<json::Number>().value;
                const auto bearing_after = maneuver.at("bearing_after").get<json::Number>().value;
                BOOST_CHECK(bearing_before >= 0. && bearing_before <= 360.);
                BOOST_CHECK(bearing_after >= 0. && bearing_after <= 360.);

                const auto type = maneuver.at("type").get<json::String>().value;
                BOOST_CHECK(!type.empty());

                // modifier is optional
                // TODO(daniel-j-h):

                // exit is optional
                // TODO(daniel-j-h):
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(test_route_response_for_locations_in_small_component)
{
    const auto args = get_args();
    auto osrm = getOSRM(args.at(0));

    using namespace osrm;

    const auto locations = get_locations_in_small_component();

    RouteParameters params;
    params.coordinates.push_back(locations.at(0));
    params.coordinates.push_back(locations.at(1));
    params.coordinates.push_back(locations.at(2));

    json::Object result;
    const auto rc = osrm.Route(params, result);
    BOOST_CHECK(rc == Status::Ok);

    const auto code = result.values.at("code").get<json::String>().value;
    BOOST_CHECK_EQUAL(code, "Ok");

    const auto &waypoints = result.values.at("waypoints").get<json::Array>().values;
    BOOST_CHECK_EQUAL(waypoints.size(), params.coordinates.size());

    for (const auto &waypoint : waypoints)
    {
        const auto &waypoint_object = waypoint.get<json::Object>();

        const auto location = waypoint_object.values.at("location").get<json::Array>().values;
        const auto longitude = location[0].get<json::Number>().value;
        const auto latitude = location[1].get<json::Number>().value;
        BOOST_CHECK(longitude >= -180. && longitude <= 180.);
        BOOST_CHECK(latitude >= -90. && latitude <= 90.);

        // TODO(daniel-j-h): we could do a Nearest request for each waypoint, verifying
        // that we did indeed not snap to the input locations inside the small component.
    }
}

BOOST_AUTO_TEST_SUITE_END()
