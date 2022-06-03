.PHONY: build clean run

build:
	@docker-compose run ros catkin init 
	@docker-compose run ros catkin build

clean:
	@docker-compose run ros catkin clean

run:
	@docker-compose run ros
