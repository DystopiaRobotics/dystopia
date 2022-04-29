# Dystopia Robotics: A Good Robot

###### Dystopia Robotics et al.

**Abstract.** A good robotics platform would allow any developer to automate any boring or deadly task. If humans can do it with the power of our brains and bodies then robots will certainly be able to as well. Today all robotics platforms outside of factories and warehouses are either cheap toys or expensive research projects. No one wants a $3000 robot that follows you down the street with a water bottle. No one is interested in a $150,000 3D mapping robot. We propose a cheap robot platform that involves deep learning systems which rely on vision, sound, and touch to act in the real world using data from simulations, few real world training examples, and self-supervised learning. The goal is to build an app platform with a developer ecosystem. Robots interacting with people in ways they understand. A cheap robot that is meant for deadly or boring work. This will counterintuitively grow the economy, creating more jobs and more money for businesses to hire more people.

## 1. Introduction

Our economy relies on billions of humans spending most productive hours working. The weakness is boring, oftentimes repetative work that most people simply perform for a pay check or because everyone else does it. Robotics is mostly in the factory or the warehouse. Many robots around the world do extremely precise work teleoperated with a human. One example is the Da Vinci Surgery Machine. A glimpse into the future is Tesla's self driving car that will within years allow the possibility of a car without a human driver at all. This shows the weakness of modern robotics is not hardware but software and data. Robots used to be only in the lab, then in the factory, then in the warehouse, now in the streets. What is needed to take robotics into the workplace and home off the streets is entirely new technology. This will lead to many more interesting jobs than the boring jobs because humans will no longer have to do boring jobs. Because boring jobs are cheaper to perform we are able to increase economic output. With more economic output robotics will counterintuitively increase the amount of interesting work that can be afforded by companies.

## 2. Wright's Law

Right now we are experiencing three major trends in robotics. One is commonly referred to as Moore's Law. This is actually Wright's Law<sup>[1](#1)</sup>. Wright's law is that the cost efficiency per unit of a product will increase a constant percentage relative to the rate of production of that product. Since Wright's Law is a function of efficiency with increased production and more fabs are being produced than ever before this trend is not likely to slow down soon. The increased number of transistors that these fabs produce allow for more matrix math and collection/storage of data. Matrix math and data are the foundation of deep learning techniques like forward propagation, back propagation, and Stochastic Gradient Descent necessary for inference and training models. These efficiency increases making up an exponential trend are a series of S-curves from different chip making innovations over time stacked on top of each other to make what looks like an exponential curve.

## 3. Algorithmic Efficiency

Another major trend in robotics is algorithmic efficiency<sup>[2](#2)</sup>. This is also an exponential trend separate from Wright's Law even if you keep the amount of transistors for your deep learning training the same. Algorithmic efficiency increases exponentially over time, meaning that every year our algorithms for deep learning are getting so much more compute efficient that we are able to do significantly more powerful vision classification with the same amount of training compute. These algorithmic efficiency increases are similar to Wright's law in the sense that the exponential trend is a series of S-curves from different innovations over time stacked on top of each other to make an exponential curve. If we make an assumption that this finding is also true for other types of deep learning related to robotics, which we hope to demonstrate practically, we can conlude that many robotics related deep learning applications will take advantage of an exponential increase in algorithmic efficiency increases this decade and next.

## 5. Economics

In the beginning the primary customer should be developers and early adopters intent on using the robot for simple repetative, boring, or deadly tasks such as teleoperation, caretaking the elderly, dangerous situations, scouting, entertainment, and basic security work. As deep learning methods mature and more data is accessible for training with more deployments and more compute with more efficient algorithms the number of jobs robots cannot do will steadily decrease. Similar to God of the Gaps, people will consistently point to fewer and fewer jobs which are the only reasons why robots will never be able to do what a human can. The robots will prove detractors wrong eventually. This will require a REST API to control the robot for developers to utilize when building applications. This will also require an App Store with a mutually beneficial governance policy to customers, developers, governments, and Dystopia Robotics. The cost of hardware should cost as little as possible without compromising on capabilities to facilitate more buyers and increase the caliber of deep learning models with a large fleet to train data from. The software should economically benefit engineers who will be developing applications for their customers. The cost of software should support developers and the development of new hardware. The cost of robotic labor to do boring tasks could be as low as a $300 deposit, $2700 at delivery, $100 per month for relevant apps, and the price of electricity. This compares favorably to a human doing boring labor.

This will ironically allow companies to do things that have never been done before, make more money than ever before, spending that money on employees doing interesting jobs that a robot cannot yet do such as YouTube marketing, design, or creating new products. Just as farming and foraging is no longer an occupation for the vast majority of our species, so too this century will boring and dangerous jobs be something only a small fraction of our society participates in. By the end of this century the majority of our economy will be robots interacting with other robots leaving humans to focus on interesting work and propagating the species. Those who say robots will take our jobs are under the outtaded impression that jobs are finite and static. There are infinite possible jobs and the nature of work is constantly changing. Just as we no longer need to forage for berries so too will we never need to do a boring or dangerous job again. Just as a computer is a bicycle for the mind, robots are a bicycle for the hand.

## 6. Hardware

The hardware necessary for a high quality robot is not expensive. We never want to make a $150,000 robot because it goes against the idea that robotics should be ubiquitous. The technology should not get in the way of the customer experience, and that includes the price as well as the hardware and software. Product is more important than marketing. The best part is no part. The best process is no process. If we cannot make a compelling product with a small cheap feature set such as GPS, cameras, microphones, hall effect sensors, force sensors, speakers, batteries, inference chips, data storage, and inductive charging then we have failed and hopefully someone's children can pick up the yoke of what we have to accomplish for the species.

## 7. Software

The software necessary for a high quality robot should not be based on robotics methods of the past. First principles should be a cornerstone of the company, in our personal lives as well as inside Dystopia Robotics. Nothing should be taken for granted. No sacred cows allowed. And data as well as the customer experience should backup our reasoning behind every software change. The best software is less software. The best process is no process. The best interaction method is no interaction. Over the air updates via wifi/5G should improve our customers lives by surprising and delighting them. Self charging automatically should allow the customer to never think about charging the robot. Our tooling will need to revolve around deep learning applications. Data cleanup, software made for deep learning models that can get larger, rapid retraining, continuous testing, and the ability to upgrade architectures is key to evolving the robot.

## 8. Walking

Rapid Motor Adaptation is the state of the art in robotics for walking on arbitrary surfaces as recently as 2021<sup>[3](#3)</sup>. The trend with walking in robotics is to do as much data generation in simulation as possible for training deep learning models. This will not slow down. Getting data from the real world alone is not scalable when deployments are not able to generate enough data for rapid and efficient training. Methods that rely on human programmers to manually make inverse kinematics functions are brittle and should be used minimally and consistently replaced as deep learning advances. When real world data is used for training self supervised learning should be used as much as possible because human labeling is expensive and error prone. Training in simulation should also include making sure the robot will not do anything that destroys the robot or that is unsafe. With the double exponential trend made up by Wright's Law and Algorithmic Efficiency walking using deep learning models trained in simulation with minimal real world examples will improve dramatically this decade.

## 9. Manipulation

Methods such as A Framework for Efficient Robotic Manipulation<sup>[4](#4)</sup> show that, given only 10 demonstrations, a single robotic arm can learn sparse-reward manipulation policies from pixels, such as reaching, picking, moving, pulling a large object, flipping a switch, and opening a drawer in just 15-50 minutes of real-world training time. The small number of real world demonstrations will be necessary in the beginning. As more deployments happen we can ask the fleet for specific types of teleoperated real world demonstration examples for training future deep learning methods. We can also have human teleoperators in the loop potentially for higher priced software to help facilitate the deep learning models. With the double exponential trend made up by Wright's Law and Algorithmic Efficiency manipulation using deep learning models trained in simulation with minimal real world examples will improve dramatically this decade.

## 10. Interaction

Methods similar to Language Models as Zero-Shot Planners: Extracting Actionable Knowledge for Embodied Agents<sup>[5](#5)</sup> mean that we can translate human commands into steps, seek confirmation for those steps, and translate those steps into robotics functions with input parameters. A simulated home environment with simulated people doing simulated tasks is obviously going to need a lot of work to translate to the real world, but similar methods will allow robots in the near future do simple tasks with simple commands. With the double exponential trend made up by Wright's Law and Algorithmic Efficiency interaction using deep learning models will dramatically improve this decade. Causal language models trained using self-supervised learning on internet data are set to improve dramatically this decade. Then translating those steps into robotics functions with pre-trained masked language models and simulations with minimal real world examples will also improve dramatically this decade.

## 11. Vision

Methods similar to NeRF for generating 3D maps from 2D camera data allow for inexpensive embedded system hardware and will improve with time<sup>[6](#6)</sup>. Many robot companies currently rely on expensive hardware such as LIDAR. Evidence is mounting with mobileye, Tesla, and Covariant that vision only robots are able to improve dramatically with time as we exponentially scale compute and algortihmic efficiency in vision. With the double exponential trend made up by Wright's Law and Algorithmic Efficiency vision using deep learning models will dramatically improve this decade.

## 12. Data

The data stored on robots should not be accessible even with physical access to the hard drive ideally using encryption with private keys not stored on the robot. A good robot will not compromise a users data if physically stolen. The infrastructure on the backend should be able to use the fleet of robots to pull data for training examples only, not for spying on individuals or selling data.

## 13. Safety

The cornerstone of our robotics company should be caring about people. If something is good for the company but bad for the people we should never do it. Nothing is irreplacable except people. Our mission is to make a robot that saves lives in the long term. We won't be able to save every single person just as a doctor can't save all their patients and we shouldn't try because the goal is to improve rapidly which will save more precious lives in the long run. Supervision of the robot should be required at all times in prototype and beta phase. A simple voice command or emergency kill switch should result in the robot laying down and turning off immediately. The robot should recognize and avoid fire as well as possible. Hall effect sensors in the arms should detect strong magnetic fields coming from high current which the robot should avoid manipulating. Extensive integration testing and unit testing for safety around humans and objects should be our first priority in our CI/CD pipeline run against all code before release to customers to avoid the robot damaging people or itself. We simply do not sell or share any data with third parties. This is to protect customers. Developers should always strive to be good stewards that are not to be trusted by default with raw data from customers. We should make developer APIs for their apps to interact with the world, but allowing direct access to raw image or voice data is dangerous and not ideal.

## 14. Culture

We should strive to hire the best aligned with the mission of saving lives with robotics. This doesn't mean the smartest or the ones who are interested in being prestigious researchers but those who are most interested in learning the gritty details of making a new product and sharing it with millions of people. Manufacturing will be important and the product should speak for itself. Our marketing messages should be crisp and ideally non-existent because our products should make money for people and for ourselves so when we are ready to sell people will want to buy without much convincing. A reliable robot will not be built by reaching 80% against standard benchmarks for the admiration of our colleagues and then moving on to the next problem. Robots yearn for the long tail of nines in simulation transfered the to the real world.

Diversity makes us better products which makes us more money. And it's the right thing to do anyway. Many executives say they only care about hiring the best but they surround themselves with people who look just like them, so they think they are the best when they're really not. We should strive to surround ourselves with neurodiverse people from all countries and backgrounds united by the mission of robots that save lives. This includes partnering with media outlets and non-engineers who share the mission. If new methods come along which are better we should always scrap the old methods and be willing to fosture a culture where it is safe to fail as long as you learned. Risk means failure sometimes, perseverance requires a quiet long march, and there is no guarantee we won't fail. Our mission will literally be the most important in the world, to save lives and keep the species safe so we can keep propagating.

## 15. Risks

The biggest risk to robotics progress is regulation. Incentives in multiple directions must be aligned for regulations to not kill innovation. A good robot gives money to governments and grows the economy. A good robot does right by the people by improving their lives. If these incentives are aligned robotics and people will prosper together. Fundamentally robots are not a tool of war or religion. Imagine millions of robots with no regard for human life. Imagine millions of robots making sure women can't leave thier homes. Governance of the app stores should include input from a majority of people. There needs to be a process for having apps vetted and accepted on the store and pulling down bad actors. State actors should not be allowed to run roughshot on the robot's infrastructure. The only way to do this is to severely degrade the ability of Advanced Persistent Threats from all nations, including our nation of origin. Heavy security and personal transparency reduces the ability for APTs to blackmail and compromise employees working on the robot. Alignment with human values is alignment with the mission of robots that save lives in the long term. Societal norms and objectives change through the ages. A good robot changes it's values to keep the planet's species safe.

## 16. Conclusion

Algorithmic efficiency and Wright's Law result in a double exponential increase in deep learning model capabilities over time. This means objectively we are able to make more powerful deep learning models on more compute with the same amount of battery. This is leading to a renaissance of robotics this decade and next that is not comparable to any other period in history. Our intention is for robotics to lead the world into a better place than we found it.

# References

###### [<a name="1">1</a>] Ark Invest, "What is Wright's Law?", [https://ark-invest.com/wrights-law/](https://ark-invest.com/wrights-law/), 2019.
###### [<a name="2">2</a>] OpenAI, "Measuring the Algorithmic Efficiency of Neural Networks", [https://arxiv.org/pdf/2005.04305.pdf](https://arxiv.org/pdf/2005.04305.pdf), 2020.
###### [<a name="3">3</a>] Facebook and UC Berkeley, "RMA: Rapid Motor Adaptation for Legged Robots", [https://arxiv.org/pdf/2107.04034.pdf](https://arxiv.org/pdf/2107.04034.pdf), 2021.
###### [<a name="4">4</a>] UC Berkeley, "A Framework for Efficient Robotic Manipulation", [https://arxiv.org/pdf/2012.07975.pdf](https://arxiv.org/pdf/2012.07975.pdf), 2020.
###### [<a name="5">5</a>] UC Berkeley/Carnegie Mellon/Google, "Language Models as Zero-Shot Planners: Extracting Actionable Knowledge for Embodied Agents", [https://arxiv.org/pdf/2201.07207.pdf](https://arxiv.org/pdf/2201.07207.pdf), 2022.
###### [<a name="6">6</a>] UC Berkeley/UCSD/Google, "Representing Scenes as Neural Radiance Fields for View Synthesis", [https://www.matthewtancik.com/nerf](https://www.matthewtancik.com/nerf), 2021.

# License
Copyright (C) 2021-2022 Dystopia Robotics Inc. chase@dystopiarobotics.com

This file is part of the Dystopia Robotics project.

The Dystopia Robotics project can not be copied and/or distributed without the express
permission of Dystopia Robotics chase@dystopiarobotics.com

## For local development

Install homebrew

```
sudo /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
```

For Unix/Linux OS you might go to the official Terraform site and download bin-file with software.  
  
Install docker  
Download here https://www.docker.com/products/docker-desktop  
Follow the instructions to install it and start docker desktop  
  
Install Postman  
Download here https://www.postman.com/downloads/  
Follow the instructions to install it  

Install terraform  
```
brew tap hashicorp/tap
brew install hashicorp/tap/terraform
```

This was my version of terraform and my providers
chase@Chases-MacBook-Pro dystopia % terraform --version  
Terraform v0.14.9
+ provider registry.terraform.io/hashicorp/aws v3.33.0
+ provider registry.terraform.io/hashicorp/template v2.2.0

Install aws cli
```
brew install awscli
```

Install ansible
```
brew install ansible
```

Install postgres
```
brew install postgres
```

Install python annd packages for testing the API
```
brew install python3
pip3 install requests
pip3 install coolname
```

## Create an AWS Account

Go through the steps to setup a new AWS account for attaching to Terraform Cloud and login to your root account to get to your [AWS account console](https://us-east-1.console.aws.amazon.com/console/home?region=us-east-1#)
  
## Setup AWS credentials

Setup a new user called terraform in us-east-1 that has the "Select AWS credential type" option of "Access key - Programmatic access" checked [using IAM](https://us-east-1.console.aws.amazon.com/iam/home#/users$new?step=details)

The next step is to setup permissions and you should select "Attach existing policies directly" and then choose [AdministratorAccess](https://us-east-1.console.aws.amazon.com/iam/home#/policies/arn%3Aaws%3Aiam%3A%3Aaws%3Apolicy%2FAdministratorAccess)

No need to add any tags

On the review page click "Create User"

The next step you should see an AWS key pair

This AWS key pair you created for the new user called terraform in us-east-1 will be used by terraform for setting up the infrastructure in AWS and use these aws id and keys as `<your aws id>` and `<your aws key>`

Keep this key pair somewhere safe for now by clicking the "Show" link under "Secret access key" to copy and paste it somewhere else and also copy/pasting the "Access key ID"

After you have stored the "Secret access key" i.e. `<your aws key>` and "Access key ID" i.e. `<your aws id>` somewhere safe click close to finish AWS credential setup

## Gain access to a foundation model

Setup a GPT-3 OpenAI access account and use the key found here https://beta.openai.com/docs/developer-quickstart/your-api-keys as `<your OpenAI API key>`

## Make a private and public ssh key pair
```
ssh-keygen -t rsa -b 4096 -C "chase.brignac@gmail.com"
```
Press enter a few times to accept defaults until the command is done

Start the ssh agent in the background
```
eval "$(ssh-agent -s)"
```

Add SSH private key to the ssh-agent
```
ssh-add ~/.ssh/id_rsa
```

## Update your public key

Put your public ssh key (~/.ssh/id_rsa.pub) in the resource "aws_key_pair" "dystopiarobotics" section of the main.tf terraform file as the string value of public_key in quotation marks so you can attempt connecting to resources later
  
You also need to update the IP address allowed to attempt access to resources
you will find this in the following section:  
"aws_security_group" "dystopiarobotics_public"

## Setup Terraform Cloud

Setup a [terraform cloud organization](https://app.terraform.io/app/organizations/new)
  
Once you have verified your email address by clicking the link sent to the email you used to sign up, [setup a workspace](https://app.terraform.io/app/dystopiarobotics/workspaces/new)

integrate your workspace with your github repo by choosing a type of "Version Control Workflow"

Choose github.com as your version control provider and authorize Terraform to connect with your github account

If the correct repos do not appear to be an option you may need to add an organization repo

Export your sensitive information as environment variables in terraform cloud located [here](https://app.terraform.io/app/dystopiarobotics/workspaces/dystopiarobotics/variables) under `Environment Variables`

Your environment variables are all sensitive so be sure when you add a variable key value pair you check "sensitive" checkbox
```
TF_VAR_openai_api_key = <your OpenAI API key>
AWS_ACCESS_KEY_ID = <your aws id>
AWS_SECRET_ACCESS_KEY = <your aws key>
TF_VAR_aws_access_key_id = <your aws id>
TF_VAR_aws_secret_access_key = <your aws key>
```

Now when you push to github terraform cloud will automatically attempt an apply, show you the resulting changes, and ask for your manual confirmation of a terraform plan before a terraform apply is run https://app.terraform.io/app/dystopiarobotics/workspaces/dystopiarobotics/runs  
  
Then state is updated and managed in the cloud automatically for you here https://app.terraform.io/app/dystopiarobotics/workspaces/dystopiarobotics/states

Multiple people can use this, you don't always need to terraform apply, and you don't need to manage sensitive passwords or state on your local machine  
  
Wait for terraform apply to finish and you should have a green output in your run if all goes well

## Enable ssh key agent forwarding and login to the private instance to setup

Open up your ssh config and edit it making sure to use the IP addresses you just found for your instances in EC2
```
nano ~/.ssh/config
```

```
Host *
  AddKeysToAgent yes
  UseKeychain yes
  IdentityFile ~/.ssh/id_rsa
Host 3.237.80.32
  HostName 3.237.80.32
  ForwardAgent yes
  IdentityFile ~/.ssh/id_rsa
  User ubuntu
Host 172.17.0.41
  User ubuntu
  IdentityFile ~/.ssh/id_rsa
  ProxyCommand ssh -W %h:%p 3.237.80.32
```
Close the file

Make sure the config file isn't accessible to everyone
```
chmod 600 ~/.ssh/config
```

Now you will login to your private machine without using a bastion with AWS Systems Manager in the AWS console
```
ssh -A -i "~/.ssh/id_rsa" ubuntu@public.dystopiarobotics.com
```
If you get the error Host key verification failed. you need to open your ~/.ssh/known_hosts file and empty it

This error means that someone may have replaced the instance with another one and is trying to trick you

Usually the simpler explanation is that you yourself or the local infrastructure admin have replaced the instance

But be security minded and be careful
```
ssh -o StrictHostKeyChecking=no -i "~/.ssh/id_rsa" ubuntu@private.dystopiarobotics.com
```
ssm and aws_instance user_data have put a zipped up version of dystopiarobotics on the instance for your convenience
```
cd /data
```

Press ctrl+D once when you setup the database to get back to your local development machine

## Setup github secrets

Start a new repo in github called dystopiarobotics

If you are not forking the dystopia Github repo and want to initialize the dystopia folder as a git repo
```
git init
git branch -m main
```

Make sure to setup ssh keys in github and locally using [these instructions](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh)

Or use HTTPS

Push to github and setup your repo in github to allow Github actions
```
git add .
git commit -m "initial commit"
git push origin main
```

Make sure you add AWS_ACCESS_KEY_ID with a value of `<your aws id>` and AWS_SECRET_ACCESS_KEY with a value of `<your aws key>` in your Github secrets, for example Dystopia Robotics secrets are found [here](https://github.com/chasebrignac/dystopiarobotics/settings/secrets/actions)

## Set github workflows environment variables

I have my environment variables set in github workflow but you will need to put your own values in, my settings are [found here](https://github.com/chasebrignac/dystopiarobotics/blob/main/.github/workflows/aws.yml)

## Run a Github action so that you can push an image to ECR and deploy automatically

When you are ready to zip up some of the scripts to put on the private instance run this command
```
rm dystopiarobotics.tar.gz && rsync -a *.sql dystopiarobotics && rsync -a *.py dystopiarobotics && rsync -a *.yml dystopiarobotics && rsync -a *.txt dystopiarobotics && rsync -a topics.csv dystopiarobotics && rsync -a Dockerfile dystopiarobotics && rsync -a clf.joblib dystopiarobotics && rsync -a templates dystopiarobotics && rsync -a *.json dystopiarobotics && tar -zcvf dystopiarobotics.tar.gz dystopiarobotics && rm -rf dystopiarobotics
```

Once you push to github this also updates the version of dystopiarobotics found on the private instance after you destroy the private instance and re-run the terraform apply in terraform cloud
