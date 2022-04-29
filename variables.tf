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
  region  = "us-east-1"
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
variable "aws_secret_access_key" {
  description = "AWS access key"
  default     = "$$${aws_secret_access_key}"
  sensitive   = true
}

# The database password should be passed in at terraform runtime by using -var
variable "aws_access_key_id" {
  description = "AWS key ID"
  default     = "$$${aws_access_key_id}"
  sensitive   = true
}

# The database password should be passed in at terraform runtime by using -var
variable "openai_api_key" {
  description = "OpenAI API Key"
  default     = "$$${openai_api_key}"
  sensitive   = true
}