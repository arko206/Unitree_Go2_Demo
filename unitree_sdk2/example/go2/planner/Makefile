CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -g
LDFLAGS  :=

SRCS     := pla2exec.cc waypoint2ctrl.cc
TARGETS  := pla2exec waypoint2ctrl

.PHONY: all clean compile_commands.json

all: $(TARGETS) compile_commands.json

pla2exec: pla2exec.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

waypoint2ctrl: waypoint2ctrl.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c -o $@ $<

compile_commands.json: $(SRCS)
	@echo '[' > $@
	@echo '  {' >> $@
	@echo '    "directory": "$(CURDIR)",' >> $@
	@echo '    "file": "$(CURDIR)/pla2exec.cc",' >> $@
	@echo '    "command": "$(CXX) $(CXXFLAGS) -c -o pla2exec.o pla2exec.cc"' >> $@
	@echo '  },' >> $@
	@echo '  {' >> $@
	@echo '    "directory": "$(CURDIR)",' >> $@
	@echo '    "file": "$(CURDIR)/waypoint2ctrl.cc",' >> $@
	@echo '    "command": "$(CXX) $(CXXFLAGS) -c -o waypoint2ctrl.o waypoint2ctrl.cc"' >> $@
	@echo '  }' >> $@
	@echo ']' >> $@

clean:
	rm -f pla2exec.o waypoint2ctrl.o $(TARGETS) compile_commands.json
