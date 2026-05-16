package netbuild

import "github.com/lab1702/traffic-sim/internal/network"

// Defaults for OSM highway types when explicit tags are missing.
// Speeds are in m/s. Lanes are *per direction* (so a default of 1 on a
// two-way street produces two 1-lane edges).

type highwayDefaults struct {
	SpeedLimit  float64 // m/s
	LanesPerDir uint8
}

var defaults = map[string]highwayDefaults{
	"motorway":       {31.3, 2}, // ~70 mph
	"trunk":          {26.8, 2}, // ~60 mph
	"primary":        {22.4, 1}, // ~50 mph
	"secondary":      {17.9, 1}, // ~40 mph
	"tertiary":       {13.4, 1}, // ~30 mph
	"unclassified":   {11.2, 1},
	"residential":    {11.2, 1}, // ~25 mph
	"service":        {6.7, 1},
	"living_street":  {4.5, 1},
	"motorway_link":  {17.9, 1},
	"trunk_link":     {15.6, 1},
	"primary_link":   {13.4, 1},
	"secondary_link": {11.2, 1},
	"tertiary_link":  {11.2, 1},
	"road":           {11.2, 1}, // OSM placeholder; treat as unclassified
}

func defaultsFor(highway string) highwayDefaults {
	if d, ok := defaults[highway]; ok {
		return d
	}
	return highwayDefaults{SpeedLimit: 11.2, LanesPerDir: 1}
}

// classOf maps an OSM `highway=*` value to a network.RoadClass. Link
// variants (motorway_link, trunk_link, …) collapse onto their parent
// class so the renderer paints a ramp in the same color as its mainline.
// Unknown / unset values fall through to ClassUnknown.
func classOf(highway string) network.RoadClass {
	switch highway {
	case "motorway", "motorway_link":
		return network.ClassMotorway
	case "trunk", "trunk_link":
		return network.ClassTrunk
	case "primary", "primary_link":
		return network.ClassPrimary
	case "secondary", "secondary_link":
		return network.ClassSecondary
	case "tertiary", "tertiary_link":
		return network.ClassTertiary
	case "residential":
		return network.ClassResidential
	case "unclassified", "road":
		return network.ClassUnclassified
	case "service":
		return network.ClassService
	case "living_street":
		return network.ClassLivingStreet
	}
	return network.ClassUnknown
}
