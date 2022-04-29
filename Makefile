lint:
	terraform fmt -recursive .

push:
	make lint
	git add .
	git commit
	git push origin HEAD

init:
	(terraform init)

upgrade:
	(terraform init -upgrade)

validate:
	(terraform validate)
