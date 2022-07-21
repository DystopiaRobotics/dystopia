terraform {
  backend "remote" {
    organization = "dystopiarobotics"

    workspaces {
      name = "dystopiarobotics"
    }
  }
}

# This is to avoid terraform complaining about no region specified
provider "aws" {
  region = "us-east-1"
}

# The major region is us-east-1
variable "aws_region" {
  description = "The AWS region"
  default     = "us-east-1"
}

# The number of AZs to cover for AZ fault tolerance
variable "az_count" {
  description = "Number of AZs to cover in a given AWS region"
  default     = "2"
}

# The aws secret should be passed in at terraform runtime by using -var
# or storing in terraform secret environment variable variables
variable "aws_secret_access_key" {
  description = "AWS access key"
  default     = "$$${aws_secret_access_key}"
  sensitive   = true
}

# The aws id should be passed in at terraform runtime by using -var
# or storing in terraform secret environment variable variables
variable "aws_access_key_id" {
  description = "AWS key ID"
  default     = "$$${aws_access_key_id}"
  sensitive   = true
}

# The huggingface API key should be passed in at terraform runtime by using -var
# or storing in terraform secret environment variable variables
variable "huggingface_api_token" {
  description = "Huggingface API Token"
  default     = "$$${huggingface_api_token}"
  sensitive   = true
}

# The database password should be passed in at terraform runtime by using -var
# or storing in terraform secret environment variable variables
variable "db_password" {
  description = "Database password"
  default     = "$$${db_password}"
  sensitive   = true
}

# The minimum number of containers that should be running
variable "ecs_autoscale_min_instances" {
  default = "1"
}

# The maximum number of containers that should be running
variable "ecs_autoscale_max_instances" {
  default = "5"
}