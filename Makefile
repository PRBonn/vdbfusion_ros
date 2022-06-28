.PHONY: build clean run docker format
SOURCES := $(shell find . -regextype posix-extended -regex  ".*\.(cpp|cxx|cc|hpp|hxx|h)" | grep -vE "^./(build|3rdparty)/")

build:
	@docker-compose run ros catkin init
	@docker-compose run ros catkin build

clean:
	@docker-compose run ros catkin clean

run:
	@docker-compose run ros

docker:
	@docker-compose build ros

format:
	clang-format -i $(SOURCES)