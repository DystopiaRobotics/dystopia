### Security Groups

# public security group for load balancers and bastions
resource "aws_security_group" "dystopiarobotics_public" {
  name        = "dystopiarobotics_public"
  description = "dystopiarobotics public security group managed by Terraform"
  vpc_id      = aws_vpc.dystopiarobotics.id

  # allows ssh attempts from my IP address
  # you should change this to your IP address
  # or your corporate network
  ingress {
    protocol    = "tcp"
    from_port   = 22
    to_port     = 22
    cidr_blocks = ["69.181.183.147/32"]
  }

  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = ["0.0.0.0/0"]
  }
}

# Traffic to the ECS Cluster or private instance should only come from the ALB, DB, or elasticache
resource "aws_security_group" "dystopiarobotics_private" {
  name        = "dystopiarobotics_private"
  description = "dystopiarobotics Elastic Container Service (ECS) or Private Instance security group managed by Terraform"
  vpc_id      = aws_vpc.dystopiarobotics.id

  ingress {
    protocol  = "tcp"
    from_port = 22
    to_port   = 22

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    cidr_blocks = [aws_vpc.dystopiarobotics.cidr_block]
  }

  egress {
    protocol    = "-1"
    from_port   = 0
    to_port     = 0
    cidr_blocks = ["0.0.0.0/0"]
  }
}

### Networking and subnets

# AWS VPC for dystopiarobotics
resource "aws_vpc" "dystopiarobotics" {
  cidr_block           = "172.17.0.0/16"
  enable_dns_hostnames = true

  tags = {
    Description = "Scalable AI platform"
    Environment = "production"
    Name        = "dystopiarobotics"
  }
}

# Fetch Availability Zones in the current region
data "aws_availability_zones" "dystopiarobotics" {
}

# Create var.az_count private subnets, each in a different AZ
resource "aws_subnet" "dystopiarobotics_private" {
  count             = var.az_count
  cidr_block        = cidrsubnet(aws_vpc.dystopiarobotics.cidr_block, 8, count.index)
  availability_zone = data.aws_availability_zones.dystopiarobotics.names[count.index]
  vpc_id            = aws_vpc.dystopiarobotics.id

  tags = {
    Description = "Scalable AI platform"
    Environment = "production"
  }
}

# Create var.az_count public subnets, each in a different AZ
resource "aws_subnet" "dystopiarobotics_public" {
  count = var.az_count
  cidr_block = cidrsubnet(
    aws_vpc.dystopiarobotics.cidr_block,
    8,
    var.az_count + count.index,
  )
  availability_zone       = data.aws_availability_zones.dystopiarobotics.names[count.index]
  vpc_id                  = aws_vpc.dystopiarobotics.id
  map_public_ip_on_launch = true

  tags = {
    Description = "dystopiarobotics public subnet managed by Terraform"
    Environment = "production"
  }
}

# IGW for the public subnet
resource "aws_internet_gateway" "dystopiarobotics" {
  vpc_id = aws_vpc.dystopiarobotics.id
}

# Route the public subnet traffic through the IGW
resource "aws_route" "dystopiarobotics_internet_access" {
  route_table_id         = aws_vpc.dystopiarobotics.main_route_table_id
  destination_cidr_block = "0.0.0.0/0"
  gateway_id             = aws_internet_gateway.dystopiarobotics.id
}

# Create a NAT gateway with an EIP for each private subnet to get internet connectivity
resource "aws_eip" "dystopiarobotics" {
  count      = var.az_count
  vpc        = true
  depends_on = [aws_internet_gateway.dystopiarobotics]

  tags = {
    Description = "dystopiarobotics gateway EIP managed by Terraform"
    Environment = "production"
  }
}

# NAT gateway for internet access
resource "aws_nat_gateway" "dystopiarobotics" {
  count         = var.az_count
  subnet_id     = element(aws_subnet.dystopiarobotics_public.*.id, count.index)
  allocation_id = element(aws_eip.dystopiarobotics.*.id, count.index)

  tags = {
    Description = "dystopiarobotics gateway NAT managed by Terraform"
    Environment = "production"
  }
}

# Create a new route table for the private subnets
# And make it route non-local traffic through the NAT gateway to the internet
resource "aws_route_table" "dystopiarobotics_private" {
  count  = var.az_count
  vpc_id = aws_vpc.dystopiarobotics.id

  route {
    cidr_block     = "0.0.0.0/0"
    nat_gateway_id = element(aws_nat_gateway.dystopiarobotics.*.id, count.index)
  }

  tags = {
    Description = "dystopiarobotics gateway NAT managed by Terraform"
    Environment = "production"
  }
}

# Explicitely associate the newly created route tables to the private subnets (so they don't default to the main route table)
resource "aws_route_table_association" "dystopiarobotics_private" {
  count          = var.az_count
  subnet_id      = element(aws_subnet.dystopiarobotics_private.*.id, count.index)
  route_table_id = element(aws_route_table.dystopiarobotics_private.*.id, count.index)
}

### Instances

# instance profile for reading s3 from an EC2 instance
# which could be useful for prepoluating instances with files
resource "aws_iam_instance_profile" "dystopiarobotics_s3_private_read" {
  name = "dystopiarobotics_s3_private_read"
}

resource "aws_key_pair" "dystopiarobotics" {
  key_name   = "dystopiarobotics"
  public_key = "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAACAQC60teZFO7BuQVwHSUewqOFGo7Iko16pF/vpio8p0K4PR29KG4oaKd4lRHx0WwX5NlLTxEI5xXQWAN9sRQMz60UDnURKnGbjiy+QI/mL3Ivkt4YV6gEfGYdVChJE6bYpnmUbPn8e27JcIJkBcDEATTEZEvSWi8xNhXWOr3I4m/Jc7OOOZAk7R9roqFlsNQrOCizc543PxCLLKafwFcDNUg+h8EOO3+PVZJziAllRTx53WxYbOUZ1tSXwaiJkXSLhVmSZQU6gXuzjlUe2ZAYwW9XzQj8xvPjFJIgizJthnbFxiAn6BygM+/4YdT+SjdpG1Y3NamXgBPQPKWFX8vBkwxVIGywDqpMVlI8L1DgbU4ISVmkHj+kG8t7iX9NF73fG9M414SBpIZSO7lsXz5rHqoz7VZe5DDl5piVV/thXwaAMMm1kerF1GlWcvUxsABv4yD2DnuqMVPz77dP1abOVpRTr7NcSvQCFv4vcMO+0CAGO/RIn3vYawjLvBFEeICsd35mnWF+PDg4QiSycJpUX9wFnZKsbI+pOEfexHqseuiS+PTOgROVonC7PUzYjFbxT3SRKRsiJxNxmRtbaEjWXZpsEFjDb/ifs9K06mqTF6MqFYXVs4AhTxDuhqQ9EOBg/LG+JUIj76o4cl7VkUJxhYyP9MNO1Ze6AVl7/xmzigsEFQ== chase.brignac@example.com"
}

# private instance inside the private subnet
# reaching RDS is done through this instance
resource "aws_instance" "dystopiarobotics_private" {
  # These can be ecs optimized AMI if Amazon Linux OS is your thing
  # or you can even add an ECS compatible AMI, update instance type to t2.2xlarge
  # add to the user_data "ECS_CLUSTER= dystopiarobotics >> /etc/ecs/ecs.config"
  # and add the iam_instance_profile of aws_iam_instance_profile.dystopiarobotics_ecs.name
  # and you would then be able to use this instance in ECS
  ami           = "ami-0fa37863afb290840"
  instance_type = "t2.nano"
  subnet_id     = aws_subnet.dystopiarobotics_private[0].id

  vpc_security_group_ids = [aws_security_group.dystopiarobotics_private.id]
  key_name               = aws_key_pair.dystopiarobotics.key_name
  iam_instance_profile   = aws_iam_instance_profile.dystopiarobotics_s3_private_read.name
  depends_on             = [aws_s3_bucket_object.dystopiarobotics_private]
  user_data              = "#!/bin/bash\necho $USER\ncd /home/ubuntu\npwd\necho beginscript\nsudo apt-get update -y\nsudo apt-get install awscli -y\necho $USER\necho ECS_CLUSTER=dystopiarobotics > /etc/ecs/ecs.config\napt-add-repository --yes --update ppa:ansible/ansible\napt -y install ansible\napt install postgresql-client-common\napt-get -y install postgresql\napt-get remove docker docker-engine docker-ce docker.io\napt-get install -y apt-transport-https ca-certificates curl software-properties-common\nexport AWS_ACCESS_KEY_ID=${aws_ssm_parameter.dystopiarobotics_aws_access_key_id.value}\nexport AWS_SECRET_ACCESS_KEY=${aws_ssm_parameter.dystopiarobotics_secret_access_key.value}\nexport AWS_DEFAULT_REGION=us-east-1\naws s3 cp s3://dystopiarobotics-private/dystopiarobotics.tar.gz ./\ntar -zxvf dystopiarobotics.tar.gz\nmv dystopiarobotics data\napt install python3-pip -y\napt-get install tmux"
  # to troubleshoot your user_data logon to the instance and run this
  #cat /var/log/cloud-init-output.log

  # lifecycle {
  #   ignore_changes = [user_data]
  # }

  root_block_device {
    volume_size = "100"
    volume_type = "standard"
  }

  tags = {
    Name = "dystopiarobotics_private"
  }
}

### IAM policies

# IAM policy for reading s3 in dystopiarobotics
resource "aws_iam_policy" "dystopiarobotics_s3_private_read" {
  name        = "dystopiarobotics_s3_private_read"
  description = "Policy to allow S3 reading of bucket dystopiarobotics-private and ssm"

  policy = <<EOF
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Sid": "VisualEditor0",
            "Effect": "Allow",
            "Action": [
                "s3:GetObject",
                "ssm:GetParametersByPath",
                "ssm:GetParameters",
                "ssm:GetParameter"
            ],
            "Resource": [
                "arn:aws:s3:::dystopiarobotics-private/*",
                "arn:aws:ssm:${var.aws_region}:*:parameter/parameter/production/AWS_ACCESS_KEY_ID",
                "arn:aws:ssm:${var.aws_region}:*:parameter/parameter/production/AWS_SECRET_ACCESS_KEY",
                "arn:aws:ssm:${var.aws_region}:*:parameter/parameter/production/OPENAI_API_KEY"
            ]
        }
    ]
}
EOF
}

### S3

# dystopiarobotics s3 bucket
resource "aws_s3_bucket" "dystopiarobotics_private" {
  bucket = "dystopiarobotics-private"
  acl    = "private"

  tags = {
    Name        = "dystopiarobotics"
    Environment = "production"
  }
}

# tar-ed up dystopiarobotics directory without terraform files
resource "aws_s3_bucket_object" "dystopiarobotics_private" {
  bucket = aws_s3_bucket.dystopiarobotics_private.bucket
  key    = "dystopiarobotics.tar.gz"
  source = "dystopiarobotics.tar.gz"

  # The filemd5() function is available in Terraform 0.11.12 and later
  etag = filemd5("dystopiarobotics.tar.gz")
}

### Systems Manager

# ssm parameter group for database endpoint
resource "aws_ssm_parameter" "openai_api_key" {
  name        = "/parameter/production/OPENAI_API_KEY"
  description = "Your OpenAI API Key"
  type        = "SecureString"
  value       = var.openai_api_key
  overwrite   = "true"

  tags = {
    Name        = "dystopiarobotics"
    environment = "production"
  }
}

# ssm parameter group for user id password
resource "aws_ssm_parameter" "dystopiarobotics_aws_access_key_id" {
  name        = "/parameter/production/AWS_ACCESS_KEY_ID"
  description = "The database password"
  type        = "SecureString"
  value       = var.aws_access_key_id
  overwrite   = "true"

  tags = {
    Name        = "dystopiarobotics"
    environment = "production"
  }
}

# ssm parameter group for user secret endpoint
resource "aws_ssm_parameter" "dystopiarobotics_secret_access_key" {
  name        = "/parameter/production/AWS_SECRET_ACCESS_KEY"
  description = "The database endpoint"
  type        = "SecureString"
  value       = var.aws_secret_access_key
  overwrite   = "true"

  tags = {
    Name        = "dystopiarobotics"
    environment = "production"
  }
}
