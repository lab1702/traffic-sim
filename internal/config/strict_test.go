package config

import (
	"strings"
	"testing"
)

// TestLoadConfig_RejectsUnknownKeys: yaml.Unmarshal ignores keys it can't
// map, so a typo like "intersection:" loaded as intersection_id 0 and
// silently overrode the wrong signal.
func TestLoadConfig_RejectsUnknownKeys(t *testing.T) {
	cases := map[string]string{
		"misspelled override key": `
signals:
  - intersection: 12
    mode: off
`,
		"misspelled phase key": `
signals:
  - intersection_id: 12
    phases:
      - green_edges: [0]
        green_duration: 30
        yellow_dur: 3
`,
		"misspelled top-level key": `
turn_restriction:
  - intersection_id: 1
    ban: [u_turn]
`,
	}
	for name, yaml := range cases {
		t.Run(name, func(t *testing.T) {
			_, err := LoadConfig(writeTempYAML(t, yaml))
			if err == nil {
				t.Fatalf("unknown key must be rejected; got no error")
			}
			if !strings.Contains(err.Error(), "not found") {
				t.Errorf("error should name the unknown field; got: %v", err)
			}
		})
	}
}

// TestLoadConfig_EmptyFileIsValid: an empty file is still an empty config.
func TestLoadConfig_EmptyFileIsValid(t *testing.T) {
	cfg, err := LoadConfig(writeTempYAML(t, ""))
	if err != nil {
		t.Fatalf("empty file: %v", err)
	}
	if len(cfg.Signals) != 0 || len(cfg.TurnRestrictions) != 0 {
		t.Errorf("empty file produced %+v", cfg)
	}
}

// TestLoadConfig_RejectsNonFiniteDurations: every comparison with NaN is
// false, so NaN passed both the "negative" and the "at least one positive"
// checks; +Inf passed as a valid positive duration.
func TestLoadConfig_RejectsNonFiniteDurations(t *testing.T) {
	for _, c := range []struct{ green, yellow string }{
		{".nan", "3"},
		{"30", ".nan"},
		{".inf", "3"},
		{"30", ".inf"},
		{"-.inf", "3"},
	} {
		yaml := `
signals:
  - intersection_id: 1
    phases:
      - green_edges: [0]
        green_dur: ` + c.green + `
        yellow_dur: ` + c.yellow + `
`
		_, err := LoadConfig(writeTempYAML(t, yaml))
		if err == nil {
			t.Errorf("green_dur=%s yellow_dur=%s must be rejected; got no error", c.green, c.yellow)
			continue
		}
		if !strings.Contains(err.Error(), "finite") {
			t.Errorf("green_dur=%s yellow_dur=%s: error should mention finiteness; got: %v", c.green, c.yellow, err)
		}
	}
}
