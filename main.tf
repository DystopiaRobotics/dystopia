# route53 zone for dystopiarobotics
resource "aws_route53_zone" "zone_dystopiarobotics" {
  name = "dystopiarobotics.com"
}

# route53 record for database so that no long database endpoints need to be remembered
resource "aws_route53_record" "record_database_dystopiarobotics" {
  name    = "database.dystopiarobotics.com"
  zone_id = aws_route53_zone.zone_dystopiarobotics.id
  type    = "CNAME"
  ttl     = 30

  records = [aws_rds_cluster.dystopiarobotics.endpoint]
}

# route53 record for private EC2 instance so that no long ip addresses need to be remembered
resource "aws_route53_record" "record_private_dystopiarobotics" {
  name    = "private.dystopiarobotics.com"
  zone_id = aws_route53_zone.zone_dystopiarobotics.id
  type    = "A"
  ttl     = 30

  records = [aws_instance.dystopiarobotics_private.private_ip]
}

# route53 record for public EC2 instance so that no long ip addresses need to be remembered
resource "aws_route53_record" "record_public_dystopiarobotics" {
  name    = "public.dystopiarobotics.com"
  zone_id = aws_route53_zone.zone_dystopiarobotics.id
  type    = "A"
  ttl     = 30

  records = [aws_instance.dystopiarobotics_public.public_ip]
}

# # route53 record for short url
# resource "aws_route53_record" "short_dystopiarobotics" {
#   name    = "dystopiarobotics.com" # used to be dystopiarobotics.com
#   zone_id = aws_route53_zone.zone_dystopiarobotics.id
#   type    = "A"

#   alias {
#     name                   = aws_lb.dystopiarobotics.dns_name
#     zone_id                = aws_lb.dystopiarobotics.zone_id
#     evaluate_target_health = true
#   }
# }

# route53 record for short url
resource "aws_route53_record" "short_backend_dystopiarobotics" {
  name    = "api.dystopiarobotics.com"
  zone_id = aws_route53_zone.zone_dystopiarobotics.id
  type    = "A"

  alias {
    name                   = aws_lb.dystopiarobotics.dns_name
    zone_id                = aws_lb.dystopiarobotics.zone_id
    evaluate_target_health = true
  }
}

# # route53 record for full url
# resource "aws_route53_record" "dystopiarobotics" {
#   name    = "www.dystopiarobotics.com" # used to be www.dystopiarobotics.com
#   zone_id = aws_route53_zone.zone_dystopiarobotics.id
#   type    = "A"

#   alias {
#     name                   = aws_lb.dystopiarobotics.dns_name
#     zone_id                = aws_lb.dystopiarobotics.zone_id
#     evaluate_target_health = true
#   }
# }

# resource "aws_route53_record" "dystopiarobotics_mx" {
#   zone_id = aws_route53_zone.zone_dystopiarobotics.id
#   name    = "dystopiarobotics.com"
#   type    = "MX"

#   records = [
#     "1 ASPMX.L.GOOGLE.COM",
#     "5 ALT1.ASPMX.L.GOOGLE.COM",
#     "5 ALT2.ASPMX.L.GOOGLE.COM",
#     "10 ALT3.ASPMX.L.GOOGLE.COM",
#     "10 ALT4.ASPMX.L.GOOGLE.COM",
#     "15 lncks7jelikvghw6eywwoiuaghggwn5sq4tqqxhrp33vunj2ihaq.mx-verification.google.com."
#   ]

#   ttl = 60
# }

# resource "aws_route53_record" "dystopiarobotics_txt" {
#   zone_id = aws_route53_zone.zone_dystopiarobotics.id
#   name    = "dystopiarobotics.com"
#   type    = "TXT"

#   records = [
#     "lncks7jelikvghw6eywwoiuaghggwn5sq4tqqxhrp33vunj2ihaq.mx-verification.google.com."
#   ]

#   ttl = 60
# }

# dystopiarobotics certificate managed by Terraform
resource "aws_acm_certificate" "dystopiarobotics" {
  domain_name               = "*.dystopiarobotics.com"
  validation_method         = "DNS"
  subject_alternative_names = ["dystopiarobotics.com"]

  tags = {
    Description = "dystopiarobotics certificate managed by Terraform"
    Name        = "dystopiarobotics"
  }

  lifecycle {
    create_before_destroy = true
  }
}

# the listener needs a cert as well
resource "aws_lb_listener_certificate" "dystopiarobotics" {
  listener_arn    = aws_lb_listener.dystopiarobotics.arn
  certificate_arn = aws_acm_certificate.dystopiarobotics.arn
}

# validation record for dystopiarobotics cert
resource "aws_route53_record" "dystopiarobotics_validation" {
  name    = sort(aws_acm_certificate.dystopiarobotics.domain_validation_options[*].resource_record_name)[0]
  type    = sort(aws_acm_certificate.dystopiarobotics.domain_validation_options[*].resource_record_type)[0]
  records = [sort(aws_acm_certificate.dystopiarobotics.domain_validation_options[*].resource_record_value)[0]]
  zone_id = aws_route53_zone.zone_dystopiarobotics.id
  ttl     = "300"
}

# cert for dystopiarobotics
resource "aws_acm_certificate_validation" "dystopiarobotics" {
  certificate_arn         = aws_acm_certificate.dystopiarobotics.arn
  validation_record_fqdns = [aws_route53_record.dystopiarobotics_validation.fqdn]
}

### IAM/ECR

# ecr for holding all images
resource "aws_ecr_repository" "dystopiarobotics" {
  name                 = "dystopiarobotics"
  image_tag_mutability = "MUTABLE"

  image_scanning_configuration {
    scan_on_push = true
  }
}

# ecr admin role for dystopiarobotics
resource "aws_iam_user" "dystopiarobotics_ecr_admin" {
  name = "dystopiarobotics_ecr_admin"

  tags = {
    tag-key = "dystopiarobotics"
  }
}

# ecr admin policy for dystopiarobotics
resource "aws_iam_user_policy" "dystopiarobotics_ecr_admin" {
  name = "dystopiarobotics_ecr_admin"
  user = aws_iam_user.dystopiarobotics_ecr_admin.name

  policy = <<EOF
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Sid": "VisualEditor0",
            "Effect": "Allow",
            "Action": "ecr:*",
            "Resource": "*"
        }
    ]
}
EOF
}

# instance profile for reading s3 from an EC2 instance
# which could be useful for a bastion or prepoluating instances with files
resource "aws_iam_instance_profile" "dystopiarobotics_s3_public_read" {
  name = "dystopiarobotics_s3_public_read"
}

resource "aws_iam_instance_profile" "dystopiarobotics_s3_private_read" {
  name = "dystopiarobotics_s3_private_read"
  role = "dystopiarobotics_private_instance"
}

# instance profile for ecs
resource "aws_iam_instance_profile" "dystopiarobotics_ecs" {
  name = "dystopiarobotics_ecs"
}

# task execution ecs role for dystopiarobotics
resource "aws_iam_role" "dystopiarobotics_ecs_task_execution" {
  name = "dystopiarobotics_ecs_task_execution"

  assume_role_policy = <<EOF
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Action": "sts:AssumeRole",
            "Principal": {
               "Service": "ecs-tasks.amazonaws.com"
            },
            "Effect": "Allow",
            "Sid": ""
        }
    ]
}
EOF

  # this is necessary for hosting database passwords and hosts in AWS Systems Manager
  # for convenience and so passwords are less likely to be stored on local machines
  inline_policy {
    name = "my_inline_policy"

    policy = jsonencode({
      "Version" : "2012-10-17",
      "Statement" : [
        {
          "Effect" : "Allow",
          "Action" : [
            "ssm:GetParameters"
          ],
          "Resource" : [
            "arn:aws:ssm:${var.aws_region}:*:parameter/dystopiarobotics/production/HUGGINGFACE_API_TOKEN"
          ]
        },
        {
          "Effect" : "Allow",
          "Action" : [
            "ssmmessages:CreateControlChannel",
            "ssmmessages:CreateDataChannel",
            "ssmmessages:OpenControlChannel",
            "ssmmessages:OpenDataChannel"
          ],
          "Resource" : "*"
        }
      ]
    })
  }
}

# s3 reading role for ECS tasks
resource "aws_iam_role" "dystopiarobotics_s3_read" {
  name = "dystopiarobotics_s3_read"

  assume_role_policy = <<EOF
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Action": "sts:AssumeRole",
            "Principal": {
               "Service": "ecs-tasks.amazonaws.com"
            },
            "Effect": "Allow",
            "Sid": ""
        }
    ]
}
EOF
}

# ECS task role
resource "aws_iam_role" "dystopiarobotics_ecs" {
  name = "dystopiarobotics_ecs"

  assume_role_policy = <<EOF
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Action": "sts:AssumeRole",
            "Principal": {
               "Service": "ecs-tasks.amazonaws.com"
            },
            "Effect": "Allow",
            "Sid": ""
        }
    ]
}
EOF
}

# ECS task execution role policy attachment
resource "aws_iam_role_policy_attachment" "dystopiarobotics_ecs_task_execution" {
  role       = aws_iam_role.dystopiarobotics_ecs_task_execution.name
  policy_arn = aws_iam_policy.dystopiarobotics_ecs_task_execution.arn
}

# ECS task  role policy attachment
resource "aws_iam_role_policy_attachment" "dystopiarobotics_ecs" {
  role       = aws_iam_role.dystopiarobotics_ecs.name
  policy_arn = aws_iam_policy.dystopiarobotics_ecs.arn
}

# role policy attachment for reading s3
resource "aws_iam_role_policy_attachment" "dystopiarobotics_s3_public_read" {
  role       = aws_iam_role.dystopiarobotics_s3_read.name
  policy_arn = aws_iam_policy.dystopiarobotics_s3_public_read.arn
}

# role policy attachment for reading s3
resource "aws_iam_role_policy_attachment" "dystopiarobotics_s3_private_read" {
  role       = aws_iam_role.dystopiarobotics_s3_read.name
  policy_arn = aws_iam_policy.dystopiarobotics_s3_private_read.arn
}

# IAM policy for task execution
resource "aws_iam_policy" "dystopiarobotics_ecs_task_execution" {
  name        = "dystopiarobotics_ecs_task_execution"
  description = "Policy to allow ECS to execute tasks"

  policy = <<EOF
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Effect": "Allow",
            "Action": [
                "ecr:GetAuthorizationToken",
                "ecr:BatchCheckLayerAvailability",
                "ecr:GetDownloadUrlForLayer",
                "ecr:BatchGetImage",
                "logs:CreateLogStream",
                "logs:PutLogEvents",
                "logs:CreateLogGroup",
                "logs:DescribeLogGroups"
            ],
            "Resource": "*"
        }
    ]
}
EOF
}

# IAM policy for reading s3 in dystopiarobotics
resource "aws_iam_policy" "dystopiarobotics_s3_public_read" {
  name        = "dystopiarobotics_s3_public_read"
  description = "Policy to allow S3 reading of bucket dystopiarobotics-public"

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
                "arn:aws:s3:::dystopiarobotics-public/*"
            ]
        }
    ]
}
EOF
}

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
                "arn:aws:ssm:${var.aws_region}:*:parameter/dystopiarobotics/production/AWS_ACCESS_KEY_ID",
                "arn:aws:ssm:${var.aws_region}:*:parameter/dystopiarobotics/production/AWS_SECRET_ACCESS_KEY",
                "arn:aws:ssm:${var.aws_region}:*:parameter/dystopiarobotics/production/HUGGINGFACE_API_TOKEN"
            ]
        }
    ]
}
EOF
}

# IAM policy for ECS
resource "aws_iam_policy" "dystopiarobotics_ecs" {
  name        = "dystopiarobotics_ecs"
  description = "Policy to allow ECS access"

  policy = <<EOF
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Effect": "Allow",
            "Action": [
                "ec2:DescribeTags",
                "ecs:CreateCluster",
                "ecs:DeregisterContainerInstance",
                "ecs:DiscoverPollEndpoint",
                "ecs:Poll",
                "ecs:RegisterContainerInstance",
                "ecs:StartTelemetrySession",
                "ecs:UpdateContainerInstancesState",
                "ecs:Submit*",
                "ecr:GetAuthorizationToken",
                "ecr:BatchCheckLayerAvailability",
                "ecr:GetDownloadUrlForLayer",
                "ecr:BatchGetImage",
                "logs:CreateLogStream",
                "logs:PutLogEvents",
                "logs:CreateLogGroup",
                "logs:DescribeLogGroups"
            ],
            "Resource": "*"
        }
    ]
}
EOF
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

# Create var.az_count rds subnets, each in a different AZ
resource "aws_subnet" "dystopiarobotics_rds" {
  count = var.az_count
  cidr_block = cidrsubnet(
    aws_vpc.dystopiarobotics.cidr_block,
    8,
    2 * var.az_count + 1 + count.index,
  )
  availability_zone = data.aws_availability_zones.dystopiarobotics.names[count.index]
  vpc_id            = aws_vpc.dystopiarobotics.id

  tags = {
    Description = "dystopiarobotics RDS subnet managed by Terraform"
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

# RDS route table for dystopiarobotics
resource "aws_route_table" "dystopiarobotics_rds" {
  count  = var.az_count
  vpc_id = aws_vpc.dystopiarobotics.id

  route {
    cidr_block     = "0.0.0.0/0"
    nat_gateway_id = element(aws_nat_gateway.dystopiarobotics.*.id, count.index)
  }

  tags = {
    Description = "dystopiarobotics RDS route table managed by Terraform"
    Environment = "production"
  }
}

# Explicitely associate the newly created route tables to the private subnets (so they don't default to the main route table)
resource "aws_route_table_association" "dystopiarobotics_private" {
  count          = var.az_count
  subnet_id      = element(aws_subnet.dystopiarobotics_private.*.id, count.index)
  route_table_id = element(aws_route_table.dystopiarobotics_private.*.id, count.index)
}

resource "aws_route_table_association" "rdystopiarobotics_rds" {
  count          = var.az_count
  subnet_id      = element(aws_subnet.dystopiarobotics_rds.*.id, count.index)
  route_table_id = element(aws_route_table.dystopiarobotics_rds.*.id, count.index)
}

### RDS

# subnet used by rds
resource "aws_db_subnet_group" "dystopiarobotics" {
  name        = "dystopiarobotics"
  description = "dystopiarobotics RDS Subnet Group managed by Terraform"
  subnet_ids  = aws_subnet.dystopiarobotics_rds.*.id
}

# Security Group for resources that want to access the database
resource "aws_security_group" "dystopiarobotics_db_access" {
  vpc_id      = aws_vpc.dystopiarobotics.id
  name        = "dystopiarobotics_db_access"
  description = "dystopiarobotics allow access to RDS, managed by Terraform"

  ingress {
    # TLS (change to whatever ports you need)
    from_port = 5432
    to_port   = 5432
    protocol  = "tcp"

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    cidr_blocks = [aws_vpc.dystopiarobotics.cidr_block]
  }
}

# database security group
resource "aws_security_group" "dystopiarobotics_rds" {
  name        = "dystopiarobotics_rds"
  description = "dystopiarobotics RDS security group, managed by Terraform"
  vpc_id      = aws_vpc.dystopiarobotics.id

  //allow traffic for TCP 5432
  ingress {
    from_port = 5432
    to_port   = 5432
    protocol  = "tcp"

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    security_groups = aws_security_group.dystopiarobotics_ecs.*.id
  }

  // outbound internet access
  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }
}

# database cluster instances for dystopiarobotics
resource "aws_rds_cluster_instance" "dystopiarobotics" {
  # WARNING: Setting count to anything less than 2 reduces
  # the reliability of your system, many times an instance
  # failure has occured requiring a hot switch to a
  # secondary instance, if there is nothing to switch to
  # you may regret setting count to 1, consider reliability
  # and weigh it against infrastructure cost
  count                = 1
  cluster_identifier   = aws_rds_cluster.dystopiarobotics.id
  instance_class       = "db.r4.large"
  db_subnet_group_name = aws_db_subnet_group.dystopiarobotics.name
  engine               = "aurora-postgresql"
  engine_version       = "12.8"
}

# database cluster for dystopiarobotics
resource "aws_rds_cluster" "dystopiarobotics" {
  cluster_identifier = "dystopiarobotics"
  #availability_zones        = ["us-east-1a", "us-east-1b", "us-east-1c"]
  database_name             = "dystopiarobotics"
  master_username           = "postgres"
  master_password           = var.db_password
  db_subnet_group_name      = aws_db_subnet_group.dystopiarobotics.name
  engine                    = "aurora-postgresql"
  engine_version            = "12.8"
  vpc_security_group_ids    = [aws_security_group.dystopiarobotics_rds.id]
  skip_final_snapshot       = "true"
  final_snapshot_identifier = "foo"
  storage_encrypted         = "true"
  #snapshot_identifier      = "dystopiarobotics"
}

### Elasticache

# Security Group for resources that want to access redis
resource "aws_security_group" "dystopiarobotics_redis_access" {
  vpc_id      = aws_vpc.dystopiarobotics.id
  name        = "dystopiarobotics_redis_access"
  description = "dystopiarobotics redis access security group managed by Terraform"

  ingress {
    # TLS (change to whatever ports you need)
    from_port = 6379
    to_port   = 6379
    protocol  = "tcp"

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    cidr_blocks = [aws_vpc.dystopiarobotics.cidr_block]
  }
}

resource "aws_security_group" "dystopiarobotics_redis" {
  name        = "dystopiarobotics_redis"
  vpc_id      = aws_vpc.dystopiarobotics.id
  description = "dystopiarobotics Redis Security Group managed by Terraform"

  //allow traffic for TCP 6379
  ingress {
    from_port = 6379
    to_port   = 6379
    protocol  = "tcp"

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    security_groups = aws_security_group.dystopiarobotics_ecs.*.id
  }

  // outbound internet access
  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }
}

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
    cidr_blocks = ["0.0.0.0/0"]
  }

  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = ["0.0.0.0/0"]
  }
}

### Elasticache

# # elasticache for dystopiarobotics
# resource "aws_elasticache_subnet_group" "dystopiarobotics" {
#   name       = "dystopiarobotics"
#   subnet_ids = aws_subnet.dystopiarobotics_private.*.id
# }

# # elasticache cluster for dystopiarobotics
# resource "aws_elasticache_cluster" "dystopiarobotics" {
#   cluster_id           = "dystopiarobotics"
#   engine               = "redis"
#   node_type            = "cache.m5.large"
#   port                 = 6379
#   num_cache_nodes      = 1
#   security_group_ids   = [aws_security_group.dystopiarobotics_redis.id]
#   subnet_group_name    = aws_elasticache_subnet_group.dystopiarobotics.name
#   parameter_group_name = aws_elasticache_parameter_group.dystopiarobotics.name
# }

# # elasticache parameter group for dystopiarobotics
# resource "aws_elasticache_parameter_group" "dystopiarobotics" {
#   name   = "redis-28-dystopiarobotics"
#   family = "redis6.x"

#   parameter {
#     name  = "timeout"
#     value = "500"
#   }
# }

### AWS instances

resource "aws_key_pair" "dystopiarobotics" {
  key_name   = "dystopiarobotics"
  public_key = "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAACAQC60teZFO7BuQVwHSUewqOFGo7Iko16pF/vpio8p0K4PR29KG4oaKd4lRHx0WwX5NlLTxEI5xXQWAN9sRQMz60UDnURKnGbjiy+QI/mL3Ivkt4YV6gEfGYdVChJE6bYpnmUbPn8e27JcIJkBcDEATTEZEvSWi8xNhXWOr3I4m/Jc7OOOZAk7R9roqFlsNQrOCizc543PxCLLKafwFcDNUg+h8EOO3+PVZJziAllRTx53WxYbOUZ1tSXwaiJkXSLhVmSZQU6gXuzjlUe2ZAYwW9XzQj8xvPjFJIgizJthnbFxiAn6BygM+/4YdT+SjdpG1Y3NamXgBPQPKWFX8vBkwxVIGywDqpMVlI8L1DgbU4ISVmkHj+kG8t7iX9NF73fG9M414SBpIZSO7lsXz5rHqoz7VZe5DDl5piVV/thXwaAMMm1kerF1GlWcvUxsABv4yD2DnuqMVPz77dP1abOVpRTr7NcSvQCFv4vcMO+0CAGO/RIn3vYawjLvBFEeICsd35mnWF+PDg4QiSycJpUX9wFnZKsbI+pOEfexHqseuiS+PTOgROVonC7PUzYjFbxT3SRKRsiJxNxmRtbaEjWXZpsEFjDb/ifs9K06mqTF6MqFYXVs4AhTxDuhqQ9EOBg/LG+JUIj76o4cl7VkUJxhYyP9MNO1Ze6AVl7/xmzigsEFQ== chase.brignac@example.com"
}

# public facing instance through which maintenance work is done
# t3a.micro has enough memory to run a Duo bastion but t3a.nano will save money
resource "aws_instance" "dystopiarobotics_public" {
  ami                         = "ami-0fa37863afb290840"
  instance_type               = "t3a.micro"
  subnet_id                   = aws_subnet.dystopiarobotics_public[0].id
  associate_public_ip_address = true
  iam_instance_profile        = aws_iam_instance_profile.dystopiarobotics_s3_public_read.name
  vpc_security_group_ids      = [aws_security_group.dystopiarobotics_public.id]
  key_name                    = aws_key_pair.dystopiarobotics.key_name
  depends_on                  = [aws_s3_object.dystopiarobotics_public]
  user_data                   = "#!/bin/bash\necho $USER\ncd /home/ubuntu\npwd\necho beginscript\nexport AWS_ACCESS_KEY_ID=${aws_ssm_parameter.dystopiarobotics_aws_access_key_id.value}\nexport AWS_SECRET_ACCESS_KEY=${aws_ssm_parameter.dystopiarobotics_secret_access_key.value}\necho $AWS_SECRET_ACCESS_KEY\necho $AWS_ACCESS_KEY_ID\nexport AWS_DEFAULT_REGION=us-east-1\nsudo apt-get update -y\nsudo apt-get install awscli -y\nsudo apt-get install awscli -y\naws s3 cp s3://dystopiarobotics-public/bastion.tar.gz ./\napt-get remove docker docker-engine docker-ce docker.io\napt-get install -y apt-transport-https ca-certificates curl software-properties-common\ncurl -fsSL https://download.docker.com/linux/ubuntu/gpg  | apt-key add -\nadd-apt-repository 'deb [arch=amd64] https://download.docker.com/linux/ubuntu focal stable'\napt-get -y install docker-ce\nsystemctl start docker\napt-get install -y docker-compose\nsystemctl enable docker\ntar -zxvf bastion.tar.gz\ncd bastion/examples/compose\ndocker-compose up --build"
  # to troubleshoot your user_data logon to the instance and run this
  #cat /var/log/cloud-init-output.log

  root_block_device {
    volume_size = "20"
    volume_type = "standard"
  }

  # lifecycle {
  #   ignore_changes = [user_data]
  # }

  tags = {
    Name = "dystopiarobotics_public"
  }
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

  vpc_security_group_ids = [aws_security_group.dystopiarobotics_ecs.id]
  key_name               = aws_key_pair.dystopiarobotics.key_name
  iam_instance_profile   = aws_iam_instance_profile.dystopiarobotics_s3_private_read.name
  depends_on             = [aws_s3_bucket.dystopiarobotics_private]
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

### ECS

# ECS service for the backend
resource "aws_ecs_service" "dystopiarobotics_backend" {
  name                   = "dystopiarobotics_backend"
  cluster                = aws_ecs_cluster.dystopiarobotics.id
  task_definition        = aws_ecs_task_definition.dystopiarobotics_backend.family
  desired_count          = 1
  launch_type            = "FARGATE"
  force_new_deployment   = true
  enable_execute_command = true

  network_configuration {
    security_groups = [aws_security_group.dystopiarobotics_ecs.id]
    subnets         = aws_subnet.dystopiarobotics_private.*.id
  }

  load_balancer {
    target_group_arn = aws_lb_target_group.dystopiarobotics_backend.id
    container_name   = "dystopiarobotics-backend"
    container_port   = "8080"
  }

  depends_on = [aws_lb_listener.dystopiarobotics]

  tags = {
    Description = "dystopiarobotics Elastic Container Service managed by Terraform"
    Environment = "production"
  }

  lifecycle {
    ignore_changes = [desired_count]
  }
}

# ECS service for the frontend
# resource "aws_ecs_service" "dystopiarobotics_frontend" {
#   name                   = "dystopiarobotics_frontend"
#   cluster                = aws_ecs_cluster.dystopiarobotics.id
#   task_definition        = aws_ecs_task_definition.dystopiarobotics_frontend.family
#   desired_count          = var.app_count
#   launch_type            = "FARGATE"
#   force_new_deployment   = true
#   enable_execute_command = true

#   network_configuration {
#     security_groups = [aws_security_group.dystopiarobotics_ecs.id]
#     subnets         = aws_subnet.dystopiarobotics_private.*.id
#   }

#   load_balancer {
#     target_group_arn = aws_lb_target_group.dystopiarobotics_frontend.id
#     container_name   = "dystopiarobotics-frontend"
#     container_port   = "3000"
#   }

#   depends_on = [aws_lb_listener.dystopiarobotics]

#   tags = {
#     Description = "dystopiarobotics Elastic Container Service managed by Terraform"
#     Environment = "production"
#   }

#   lifecycle {
#     ignore_changes = [desired_count]
#   }
# }

# in case we ever want to start using reserved instances to try and save money
# resource "aws_ecs_service" "dystopiarobotics_backend_reserved" {
#   name            = "dystopiarobotics_backend_reserved"
#   cluster         = aws_ecs_cluster.dystopiarobotics.id
#   task_definition = aws_ecs_task_definition.dystopiarobotics_backend.arn
#   desired_count   = var.app_count
#   launch_type     = "EC2"
#
#   network_configuration {
#     security_groups = [aws_security_group.dystopiarobotics_ecs.id]
#     subnets         = aws_subnet.dystopiarobotics_private.*.id
#   }
#
#   load_balancer {
#     target_group_arn = aws_lb_target_group.dystopiarobotics_backend.id
#     container_name   = "dystopiarobotics_backend"
#     container_port   = "8080"
#   }
#
#   depends_on = [aws_lb_listener.dystopiarobotics]
#
#   tags = {
#     Description = "dystopiarobotics reserved Elastic Container Service managed by Terraform"
#     Environment = "production"
#   }
#
#   lifecycle {
#     ignore_changes = [desired_count]
#   }
# }

# in case we ever want to start using reserved instances to try and save money
# resource "aws_ecs_service" "dystopiarobotics_frontend_reserved" {
#   name            = "dystopiarobotics_frontend_reserved"
#   cluster         = aws_ecs_cluster.dystopiarobotics.id
#   task_definition = aws_ecs_task_definition.dystopiarobotics_frontend.arn
#   desired_count   = var.app_count
#   launch_type     = "EC2"
#
#   network_configuration {
#     security_groups = [aws_security_group.dystopiarobotics_ecs.id]
#     subnets         = aws_subnet.dystopiarobotics_private.*.id
#   }
#
#   load_balancer {
#     target_group_arn = aws_lb_target_group.dystopiarobotics_frontend.id
#     container_name   = "dystopiarobotics_frontend"
#     container_port   = "8080"
#   }
#
#   depends_on = [aws_lb_listener.dystopiarobotics]
#
#   tags = {
#     Description = "dystopiarobotics reserved Elastic Container Service managed by Terraform"
#     Environment = "production"
#   }
#
#   lifecycle {
#     ignore_changes = [desired_count]
#   }
# }

### Autoscaling

# autoscaling target for dystopiarobotics
resource "aws_appautoscaling_target" "dystopiarobotics_backend" {
  service_namespace  = "ecs"
  resource_id        = "service/${aws_ecs_cluster.dystopiarobotics.name}/${aws_ecs_service.dystopiarobotics_backend.name}"
  scalable_dimension = "ecs:service:DesiredCount"
  max_capacity       = var.ecs_autoscale_max_instances
  min_capacity       = 1
}

# # autoscaling target for dystopiarobotics
# resource "aws_appautoscaling_target" "dystopiarobotics_frontend" {
#   service_namespace  = "ecs"
#   resource_id        = "service/${aws_ecs_cluster.dystopiarobotics.name}/${aws_ecs_service.dystopiarobotics_frontend.name}"
#   scalable_dimension = "ecs:service:DesiredCount"
#   max_capacity       = var.ecs_autoscale_max_instances
#   min_capacity       = 1
# }

resource "aws_cloudwatch_metric_alarm" "dystopiarobotics_backend_memory_utilization_high" {
  alarm_name          = "dystopiarobotics_backend_memory_utilization_high"
  comparison_operator = "GreaterThanOrEqualToThreshold"
  evaluation_periods  = "1"
  metric_name         = "MemoryUtilization"
  namespace           = "AWS/ECS"
  period              = "60"
  statistic           = "Average"
  threshold           = 30

  dimensions = {
    ClusterName = aws_ecs_cluster.dystopiarobotics.name
    ServiceName = aws_ecs_service.dystopiarobotics_backend.name
  }

  alarm_actions = [aws_appautoscaling_policy.dystopiarobotics_backend_memory_utilization_high.arn]
}

# resource "aws_cloudwatch_metric_alarm" "dystopiarobotics_frontend_memory_utilization_high" {
#   alarm_name          = "dystopiarobotics_frontend_memory_utilization_high"
#   comparison_operator = "GreaterThanOrEqualToThreshold"
#   evaluation_periods  = "1"
#   metric_name         = "MemoryUtilization"
#   namespace           = "AWS/ECS"
#   period              = "60"
#   statistic           = "Average"
#   threshold           = 30

#   dimensions = {
#     ClusterName = aws_ecs_cluster.dystopiarobotics.name
#     ServiceName = aws_ecs_service.dystopiarobotics_frontend.name
#   }

#   alarm_actions = [aws_appautoscaling_policy.dystopiarobotics_frontend_memory_utilization_high.arn]
# }

# memory metric alarm
resource "aws_cloudwatch_metric_alarm" "dystopiarobotics_backend_memory_utilization_low" {
  alarm_name          = "dystopiarobotics_backend_memory_utilization_low"
  comparison_operator = "LessThanThreshold"
  evaluation_periods  = "1"
  metric_name         = "MemoryUtilization"
  namespace           = "AWS/ECS"
  period              = "60"
  statistic           = "Average"
  threshold           = 30

  dimensions = {
    ClusterName = aws_ecs_cluster.dystopiarobotics.name
    ServiceName = aws_ecs_service.dystopiarobotics_backend.name
  }

  alarm_actions = [aws_appautoscaling_policy.dystopiarobotics_backend_memory_utilization_low.arn]
}

# # memory metric alarm
# resource "aws_cloudwatch_metric_alarm" "dystopiarobotics_frontend_memory_utilization_low" {
#   alarm_name          = "dystopiarobotics_frontend_memory_utilization_low"
#   comparison_operator = "LessThanThreshold"
#   evaluation_periods  = "1"
#   metric_name         = "MemoryUtilization"
#   namespace           = "AWS/ECS"
#   period              = "60"
#   statistic           = "Average"
#   threshold           = 30

#   dimensions = {
#     ClusterName = aws_ecs_cluster.dystopiarobotics.name
#     ServiceName = aws_ecs_service.dystopiarobotics_frontend.name
#   }

#   alarm_actions = [aws_appautoscaling_policy.dystopiarobotics_frontend_memory_utilization_low.arn]
# }

# memory metric alarm
resource "aws_appautoscaling_policy" "dystopiarobotics_backend_memory_utilization_high" {
  name               = "dystopiarobotics_backend_memory_utilization_high"
  service_namespace  = aws_appautoscaling_target.dystopiarobotics_backend.service_namespace
  resource_id        = aws_appautoscaling_target.dystopiarobotics_backend.resource_id
  scalable_dimension = aws_appautoscaling_target.dystopiarobotics_backend.scalable_dimension

  step_scaling_policy_configuration {
    adjustment_type         = "ChangeInCapacity"
    cooldown                = 60
    metric_aggregation_type = "Average"

    step_adjustment {
      metric_interval_lower_bound = 0
      scaling_adjustment          = 1
    }
  }
}

# # memory metric alarm
# resource "aws_appautoscaling_policy" "dystopiarobotics_frontend_memory_utilization_high" {
#   name               = "dystopiarobotics_frontend_memory_utilization_high"
#   service_namespace  = aws_appautoscaling_target.dystopiarobotics_frontend.service_namespace
#   resource_id        = aws_appautoscaling_target.dystopiarobotics_frontend.resource_id
#   scalable_dimension = aws_appautoscaling_target.dystopiarobotics_frontend.scalable_dimension

#   step_scaling_policy_configuration {
#     adjustment_type         = "ChangeInCapacity"
#     cooldown                = 60
#     metric_aggregation_type = "Average"

#     step_adjustment {
#       metric_interval_lower_bound = 0
#       scaling_adjustment          = 1
#     }
#   }
# }

# memory metric alarm policy
resource "aws_appautoscaling_policy" "dystopiarobotics_backend_memory_utilization_low" {
  name               = "dystopiarobotics_backend_memory_utilization_low"
  service_namespace  = aws_appautoscaling_target.dystopiarobotics_backend.service_namespace
  resource_id        = aws_appautoscaling_target.dystopiarobotics_backend.resource_id
  scalable_dimension = aws_appautoscaling_target.dystopiarobotics_backend.scalable_dimension

  step_scaling_policy_configuration {
    adjustment_type         = "ChangeInCapacity"
    cooldown                = 300
    metric_aggregation_type = "Average"

    step_adjustment {
      metric_interval_upper_bound = 0
      scaling_adjustment          = -1
    }
  }
}

# # memory metric alarm policy
# resource "aws_appautoscaling_policy" "dystopiarobotics_frontend_memory_utilization_low" {
#   name               = "dystopiarobotics_frontend_memory_utilization_low"
#   service_namespace  = aws_appautoscaling_target.dystopiarobotics_frontend.service_namespace
#   resource_id        = aws_appautoscaling_target.dystopiarobotics_frontend.resource_id
#   scalable_dimension = aws_appautoscaling_target.dystopiarobotics_frontend.scalable_dimension

#   step_scaling_policy_configuration {
#     adjustment_type         = "ChangeInCapacity"
#     cooldown                = 300
#     metric_aggregation_type = "Average"

#     step_adjustment {
#       metric_interval_upper_bound = 0
#       scaling_adjustment          = -1
#     }
#   }
# }

# backend task definition
resource "aws_ecs_task_definition" "dystopiarobotics_backend" {
  depends_on = [
    aws_lb.dystopiarobotics,
    #aws_elasticache_cluster.dystopiarobotics,
    aws_rds_cluster.dystopiarobotics,
  ]
  family                   = "dystopiarobotics-backend"
  network_mode             = "awsvpc"
  requires_compatibilities = ["FARGATE"]
  cpu                      = 1024
  memory                   = 4096
  execution_role_arn       = aws_iam_role.dystopiarobotics_ecs_task_execution.arn
  task_role_arn            = aws_iam_role.dystopiarobotics_ecs_task_execution.arn

  container_definitions = jsonencode([
    {
      name      = "dystopiarobotics-backend"
      image     = "923082272114.dkr.ecr.us-east-1.amazonaws.com/dystopiarobotics:backend"
      cpu       = 256
      memory    = 512
      essential = true
      command   = ["app.py"]
      portMappings = [
        {
          containerPort = 8080
          hostPort      = 8080
        }
      ],
      secrets = [
        {
          valueFrom = "arn:aws:ssm:${var.aws_region}:923082272114:parameter/dystopiarobotics/production/HUGGINGFACE_API_TOKEN",
          name      = "HUGGINGFACE_API_TOKEN"
        }
      ],
      mountPoints = [],
      logConfiguration = {
        logDriver = "awslogs",
        options = {
          awslogs-group         = "/ecs/dystopiarobotics-backend",
          awslogs-region        = "us-east-1",
          awslogs-stream-prefix = "ecs",
          awslogs-create-group  = "true"
        }
      },
      volumesFrom = [],
      environment = []
    }
  ])
}

# # frontend task definition
# resource "aws_ecs_task_definition" "dystopiarobotics_frontend" {
#   depends_on = [
#     aws_lb.dystopiarobotics,
#     #aws_elasticache_cluster.dystopiarobotics,
#     aws_rds_cluster.dystopiarobotics,
#   ]
#   family                   = "dystopiarobotics-frontend"
#   network_mode             = "awsvpc"
#   requires_compatibilities = ["FARGATE"]
#   cpu                      = 1024
#   memory                   = 4096
#   execution_role_arn       = aws_iam_role.dystopiarobotics_ecs_task_execution.arn
#   task_role_arn            = aws_iam_role.dystopiarobotics_ecs_task_execution.arn

#   container_definitions = jsonencode([
#     {
#       name      = "dystopiarobotics-frontend"
#       image     = "923082272114.dkr.ecr.us-east-1.amazonaws.com/dystopiarobotics:frontend"
#       cpu       = 512
#       memory    = 2048
#       essential = true
#       command   = ["npm", "start"]
#       portMappings = [
#         {
#           containerPort = 3000
#           hostPort      = 3000
#         }
#       ],
#       secrets = [
#         {
#           valueFrom = "arn:aws:ssm:${var.aws_region}:923082272114:parameter/dystopiarobotics/production/HUGGINGFACE_API_TOKEN",
#           name      = "HUGGINGFACE_API_TOKEN"
#         }
#       ],
#       mountPoints = [],
#       logConfiguration = {
#         logDriver = "awslogs",
#         options = {
#           awslogs-group         = "/ecs/dystopiarobotics-frontend",
#           awslogs-region        = "us-east-1",
#           awslogs-stream-prefix = "ecs",
#           awslogs-create-group  = "true"
#         }
#       },
#       volumesFrom = [],
#       environment = []
#     }
#   ])
# }

# cloudwatch log group
resource "aws_cloudwatch_log_group" "dystopiarobotics_backend" {
  name              = "/ecs/dystopiarobotics-backend"
  retention_in_days = 30

  tags = {
    Environment = "production"
    Application = "dystopiarobotics"
  }
}

# # cloudwatch log group
# resource "aws_cloudwatch_log_group" "dystopiarobotics_frontend" {
#   name              = "/ecs/dystopiarobotics-frontend"
#   retention_in_days = 30

#   tags = {
#     Environment = "production"
#     Application = "dystopiarobotics"
#   }
# }

# This needs to be integrated completely into our container_definitions of our aws_ecs_task_definition
resource "aws_cloudwatch_log_stream" "dystopiarobotics_backend" {
  name           = "dystopiarobotics"
  log_group_name = aws_cloudwatch_log_group.dystopiarobotics_backend.name
}

# # This needs to be integrated completely into our container_definitions of our aws_ecs_task_definition
# resource "aws_cloudwatch_log_stream" "dystopiarobotics_frontend" {
#   name           = "dystopiarobotics"
#   log_group_name = aws_cloudwatch_log_group.dystopiarobotics_frontend.name
# }

# ECS cluster for dystopiarobotics
resource "aws_ecs_cluster" "dystopiarobotics" {
  name = "dystopiarobotics"

  setting {
    name  = "containerInsights"
    value = "enabled"
  }
}

# Traffic to the ECS Cluster should only come from the ALB, DB, or elasticache
resource "aws_security_group" "dystopiarobotics_ecs" {
  name        = "dystopiarobotics_ecs"
  description = "dystopiarobotics Elastic Container Service (ECS) security group managed by Terraform"
  vpc_id      = aws_vpc.dystopiarobotics.id

  ingress {
    protocol  = "tcp"
    from_port = "80"
    to_port   = "80"

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    security_groups = [aws_security_group.dystopiarobotics_lb.id]
  }

  ingress {
    protocol  = "tcp"
    from_port = 3000
    to_port   = 3000

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    security_groups = [aws_security_group.dystopiarobotics_lb.id]
  }

  ingress {
    protocol  = "tcp"
    from_port = 8080
    to_port   = 8080

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    security_groups = [aws_security_group.dystopiarobotics_lb.id]
  }

  egress {
    protocol        = "tcp"
    from_port       = "5432"
    to_port         = "5432"
    security_groups = [aws_security_group.dystopiarobotics_db_access.id]
  }

  egress {
    protocol        = "tcp"
    from_port       = "6379"
    to_port         = "6379"
    security_groups = [aws_security_group.dystopiarobotics_redis_access.id]
  }

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

### ALB

# load balancer for dystopiarobotics
resource "aws_lb" "dystopiarobotics" {
  name            = "dystopiarobotics"
  subnets         = aws_subnet.dystopiarobotics_public.*.id
  security_groups = [aws_security_group.dystopiarobotics_lb.id]
  idle_timeout    = 1800

  tags = {
    Description = "dystopiarobotics Application Load Balancer managed by Terraform"
    Environment = "production"
  }
}

# redirecting api*.dystopiarobotics.com which is an indication of a backend URL to backend target group
resource "aws_lb_listener_rule" "dystopiarobotics_backend" {
  listener_arn = aws_lb_listener.dystopiarobotics.arn
  priority     = 100

  action {
    type             = "forward"
    target_group_arn = aws_lb_target_group.dystopiarobotics_backend.arn
  }

  condition {
    host_header {
      values = ["api.*"]
    }
  }
}

# # redirecting www.dystopiarobotics.com which is an indication of a frontend URL to frontend target group
# resource "aws_lb_listener_rule" "dystopiarobotics_frontend" {
#   listener_arn = aws_lb_listener.dystopiarobotics.arn
#   priority     = 90

#   action {
#     type             = "forward"
#     target_group_arn = aws_lb_target_group.dystopiarobotics_frontend.arn
#   }

#   condition {
#     host_header {
#       values = ["www.dystopiarobotics.com"]
#     }
#   }
# }

# target group for dystopiarobotics backend
resource "aws_lb_target_group" "dystopiarobotics_backend" {
  name        = "dystopiarobotics-backend"
  port        = 8080
  protocol    = "HTTP"
  vpc_id      = aws_vpc.dystopiarobotics.id
  target_type = "ip"
  slow_start  = 60

  health_check {
    interval = 60
    timeout  = 10
    path     = "/"
    matcher  = "200"
  }

  tags = {
    Description = "dystopiarobotics Application Load Balancer target group managed by Terraform"
    Environment = "production"
  }
}

# # target group for dystopiarobotics frontend
# resource "aws_lb_target_group" "dystopiarobotics_frontend" {
#   name        = "dystopiarobotics-frontend"
#   port        = 3000
#   protocol    = "HTTP"
#   vpc_id      = aws_vpc.dystopiarobotics.id
#   target_type = "ip"
#   slow_start  = 60

#   health_check {
#     interval = 60
#     timeout  = 10
#     path     = "/"
#     matcher  = "200"
#   }

#   tags = {
#     Description = "dystopiarobotics Application Load Balancer target group managed by Terraform"
#     Environment = "production"
#   }
# }

# security group for dystopiarobotics load balancer
resource "aws_security_group" "dystopiarobotics_lb" {
  name        = "dystopiarobotics_lb"
  description = "dystopiarobotics load balancer security group managed by Terraform"
  vpc_id      = aws_vpc.dystopiarobotics.id

  ingress {
    protocol  = "tcp"
    from_port = 443
    to_port   = 443

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    cidr_blocks = ["0.0.0.0/0"]
  }

  ingress {
    protocol  = "tcp"
    from_port = 80
    to_port   = 80

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    cidr_blocks = ["0.0.0.0/0"]
  }

  ingress {
    protocol  = "tcp"
    from_port = 3000
    to_port   = 3000

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    cidr_blocks = ["0.0.0.0/0"]
  }

  ingress {
    protocol  = "tcp"
    from_port = 8080
    to_port   = 8080

    # Please restrict your ingress to only necessary IPs and ports.
    # Opening to 0.0.0.0/0 can lead to security vulnerabilities.
    cidr_blocks = ["0.0.0.0/0"]
  }

  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = ["0.0.0.0/0"]
  }
}

# Redirect all traffic from the ALB to the target group
resource "aws_lb_listener" "dystopiarobotics" {
  load_balancer_arn = aws_lb.dystopiarobotics.id
  port              = "443"
  protocol          = "HTTPS"
  ssl_policy        = "ELBSecurityPolicy-TLS-1-2-2017-01"
  certificate_arn   = aws_acm_certificate.dystopiarobotics.arn

  default_action {
    target_group_arn = aws_lb_target_group.dystopiarobotics_backend.id
    type             = "forward"
  }
}

# listener for http to be redirected to https
resource "aws_lb_listener" "dystopiarobotics_http" {
  load_balancer_arn = aws_lb.dystopiarobotics.id
  port              = "80"
  protocol          = "HTTP"

  default_action {
    type = "redirect"

    redirect {
      port        = "443"
      protocol    = "HTTPS"
      status_code = "HTTP_301"
    }
  }
}

### S3

resource "aws_s3_bucket_acl" "dystopiarobotics_private_acl" {
  bucket = aws_s3_bucket.dystopiarobotics_private.id
  acl    = "private"
}

resource "aws_s3_bucket_acl" "dystopiarobotics_public_acl" {
  bucket = aws_s3_bucket.dystopiarobotics_public.id
  acl    = "private"
}

# dystopiarobotics s3 bucket
resource "aws_s3_bucket" "dystopiarobotics_public" {
  bucket = "dystopiarobotics-public"

  tags = {
    Name        = "dystopiarobotics"
    Environment = "production"
  }
}

# dystopiarobotics s3 bucket
resource "aws_s3_bucket" "dystopiarobotics_private" {
  bucket = "dystopiarobotics-private"

  tags = {
    Name        = "dystopiarobotics"
    Environment = "production"
  }
}

# bastion
resource "aws_s3_object" "dystopiarobotics_public" {
  bucket = aws_s3_bucket.dystopiarobotics_public.bucket
  key    = "bastion.tar.gz"
  source = "bastion.tar.gz"

  # The filemd5() function is available in Terraform 0.11.12 and later
  etag = filemd5("bastion.tar.gz")
}

# tar-ed up dystopiarobotics directory without terraform files
resource "aws_s3_object" "dystopiarobotics_private" {
  bucket = aws_s3_bucket.dystopiarobotics_private.bucket
  key    = "dystopiarobotics.tar.gz"
  source = "dystopiarobotics.tar.gz"

  # The filemd5() function is available in Terraform 0.11.12 and later
  etag = filemd5("dystopiarobotics.tar.gz")
}

### Systems Manager

# ssm parameter group for database endpoint
resource "aws_ssm_parameter" "dystopiarobotics_hugging_face_token" {
  name        = "/dystopiarobotics/production/HUGGINGFACE_API_TOKEN"
  description = "Your Hugging Face API Key"
  type        = "SecureString"
  value       = var.huggingface_api_token
  overwrite   = "true"

  tags = {
    Name        = "dystopiarobotics"
    environment = "production"
  }
}

# ssm parameter group for user id password
resource "aws_ssm_parameter" "dystopiarobotics_aws_access_key_id" {
  name        = "/dystopiarobotics/production/AWS_ACCESS_KEY_ID"
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
  name        = "/dystopiarobotics/production/AWS_SECRET_ACCESS_KEY"
  description = "The database endpoint"
  type        = "SecureString"
  value       = var.aws_secret_access_key
  overwrite   = "true"

  tags = {
    Name        = "dystopiarobotics"
    environment = "production"
  }
}